// heater.c
#include "heater.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/spi.h"
#include <stdio.h>
#include <math.h>
#include "pico/time.h"

#ifndef HEATER_PWM_WRAP
#define HEATER_PWM_WRAP 10000u
#endif

// PI controller constants (tune for your heater)
static const float PI_KP = 1.2f;
static const float PI_KI = 0.05f;

#define SOFT_START_MS 5000u     // ramp duration after heater_start() (tuneable)
#define SOFT_START_MAX_PC 30.0f // maximum allowed duty% at start ramp (percent)

static mutex_t *g_spi_mutex = NULL;
static bool g_initialized = false;
static uint8_t g_pwm_pin = 2;
static uint8_t g_tc_cs_pin = 5;
static uint8_t g_control_mode = 1; // 0 = on/off, 1 = PI
static float g_target_c = 25.0f;
static float g_integral = 0.0f;
static float g_duty_percent = 0.0f;
static bool g_have_tc = false;

// New: whether the heater control is "armed" (only when true will periodic
// control compute and apply PWM). Starts false so device remains safe at boot.
static bool g_running = false;

// Last-read thermocouple temperature cache (NAN when not read/available)
static float g_last_tc_temp = NAN;

static uint64_t g_start_ms = 0;

void heater_register_spi_mutex(mutex_t *m)
{
    g_spi_mutex = m;
}

static inline void spi_lock(void)
{
    if (g_spi_mutex)
        mutex_enter_blocking(g_spi_mutex);
}
static inline void spi_unlock(void)
{
    if (g_spi_mutex)
        mutex_exit(g_spi_mutex);
}

void heater_init_if_enabled(uint8_t pwm_pin, uint8_t tc_cs_pin, uint8_t control_mode, float target_c)
{
    g_pwm_pin = pwm_pin;
    g_tc_cs_pin = tc_cs_pin;
    g_control_mode = control_mode;
    g_target_c = target_c;
    g_integral = 0.0f;
    g_duty_percent = 0.0f;
    g_have_tc = false;
    g_running = false; // start in stopped state (safe)
    g_last_tc_temp = NAN;

    // Setup TC CS pin (idle high)
    gpio_init(g_tc_cs_pin);
    gpio_set_dir(g_tc_cs_pin, GPIO_OUT);
    gpio_put(g_tc_cs_pin, 1);

    // Setup PWM pin
    gpio_set_function(g_pwm_pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(g_pwm_pin);
    pwm_set_wrap(slice, HEATER_PWM_WRAP);
    pwm_set_chan_level(slice, pwm_gpio_to_channel(g_pwm_pin), 0);
    pwm_set_enabled(slice, true);

    g_initialized = true;
}

// Read MAX31855-like 32-bit value and convert to °C.
// Caller must ensure SPI bus is safe (we lock inside).
static float read_tc_once(void)
{
    if (!g_initialized)
        return NAN;

    // Acquire SPI bus
    spi_lock();

    uint8_t buf[4] = {0};
    // Toggle CS and read 4 bytes
    gpio_put(g_tc_cs_pin, 0);
    busy_wait_us(2);
    spi_set_format(spi0, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    spi_read_blocking(spi0, 0x00, buf, 4);
    gpio_put(g_tc_cs_pin, 1);

    spi_unlock();

    uint32_t raw = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8) | ((uint32_t)buf[3]);

    // Basic fault detection: some MAX31855 variants set low bits; treat as fault if lower bits non-zero
    if (raw & 0x00010000u) // check fault bit variant (robust check)
        return NAN;

    int32_t tc_bits = (int32_t)((raw >> 18) & 0x3FFF);
    if (tc_bits & 0x2000)
        tc_bits |= ~0x3FFF; // sign extend for 14-bit signed
    float tc_c = ((float)tc_bits) * 0.25f;
    return tc_c;
}

void heater_set_target_c(float t)
{
    g_target_c = t;
    // resetting integral might help after big setpoint jump
    g_integral = 0.0f;
}

float heater_get_target_c(void) { return g_target_c; }

void heater_force_off(void)
{
    g_duty_percent = 0.0f;
    if (g_initialized)
    {
        uint slice = pwm_gpio_to_slice_num(g_pwm_pin);
        pwm_set_chan_level(slice, pwm_gpio_to_channel(g_pwm_pin), 0);
    }
}

bool heater_present_tc(void) { return g_have_tc; }

void heater_update_periodic(uint32_t dt_ms)
{
    if (!g_initialized)
        return;

    // Read thermocouple (always attempt read, so g_have_tc reflects actual state)
    float temp = read_tc_once();
    if (isnan(temp))
    {
        // failed to read tc; mark missing
        g_have_tc = false;
        g_last_tc_temp = NAN;
        // If not running, just return quickly
        if (!g_running)
        {
            // ensure PWM remains off
            if (g_initialized)
            {
                uint slice = pwm_gpio_to_slice_num(g_pwm_pin);
                pwm_set_chan_level(slice, pwm_gpio_to_channel(g_pwm_pin), 0);
            }
            return;
        }
        // if running but TC read failed, we should be conservative and turn heater off
        g_running = false;
        g_integral = 0.0f;
        g_duty_percent = 0.0f;
        if (g_initialized)
        {
            uint slice = pwm_gpio_to_slice_num(g_pwm_pin);
            pwm_set_chan_level(slice, pwm_gpio_to_channel(g_pwm_pin), 0);
        }
        printf("[HEATER] TC read failed — disarming heater for safety\n");
        return;
    }

    // TC read succeeded
    g_have_tc = true;
    g_last_tc_temp = temp;

    // Debug/log current reading
    // printf("[HEATER] read temp=%.2f target=%.2f running=%d integral=%.2f duty=%.2f\n",
    //        temp, g_target_c, (int)g_running, g_integral, g_duty_percent);

    // If not running, do NOT compute or apply control — only return after read
    if (!g_running)
    {
        // make sure PWM is zero
        if (g_initialized)
        {
            uint slice = pwm_gpio_to_slice_num(g_pwm_pin);
            pwm_set_chan_level(slice, pwm_gpio_to_channel(g_pwm_pin), 0);
        }
        return;
    }

    // Running: perform control (PI or on/off)
    if (g_control_mode == 1)
    {
        // PI
        float err = g_target_c - temp;
        float dt = (float)dt_ms / 1000.0f;
        g_integral += err * dt;
        float out = PI_KP * err + PI_KI * g_integral;
        if (out < 0.0f)
            out = 0.0f;
        if (out > 100.0f)
            out = 100.0f;

        // apply soft-start cap (if recently started)
        if (g_running && g_start_ms != 0)
        {
            uint64_t since = to_ms_since_boot(get_absolute_time()) - g_start_ms;
            if (since < SOFT_START_MS)
            {
                // linear ramp of cap from 0 -> SOFT_START_MAX_PC over SOFT_START_MS
                float frac = (float)since / (float)SOFT_START_MS;
                float cap = SOFT_START_MAX_PC * frac;
                if (out > cap)
                    out = cap;
            }
            else
            {
                // ramp finished, clear start marker for slight efficiency
                g_start_ms = 0;
            }
        }

        g_duty_percent = out;
    }
    else
    {
        // on/off with hysteresis +/-1°C
        float out = 0.0f;
        if (temp < g_target_c - 1.0f)
            out = 100.0f;
        else if (temp > g_target_c + 1.0f)
            out = 0.0f;

        // apply soft-start cap same as above
        if (g_running && g_start_ms != 0)
        {
            uint64_t since = to_ms_since_boot(get_absolute_time()) - g_start_ms;
            if (since < SOFT_START_MS)
            {
                float frac = (float)since / (float)SOFT_START_MS;
                float cap = SOFT_START_MAX_PC * frac;
                if (out > cap)
                    out = cap;
            }
            else
            {
                g_start_ms = 0;
            }
        }
        g_duty_percent = out;
    }

    // Apply PWM
    if (g_initialized)
    {
        uint slice = pwm_gpio_to_slice_num(g_pwm_pin);
        uint level = (uint)((g_duty_percent / 100.0f) * (float)HEATER_PWM_WRAP);
        pwm_set_chan_level(slice, pwm_gpio_to_channel(g_pwm_pin), level);
    }
}

// Start heater control
void heater_start(void)
{
    g_integral = 0.0f; // avoid integral windup causing immediate large output
    g_duty_percent = 0.0f;
    g_running = true;
    g_start_ms = to_ms_since_boot(get_absolute_time());
    // Debug print
    printf("[HEATER] START requested: target=%.2f\n", g_target_c);
}

// Stop heater: disarm controller, clear integral and force PWM off
void heater_stop(void)
{
    g_running = false;
    g_integral = 0.0f;
    g_duty_percent = 0.0f;
    if (g_initialized)
    {
        uint slice = pwm_gpio_to_slice_num(g_pwm_pin);
        pwm_set_chan_level(slice, pwm_gpio_to_channel(g_pwm_pin), 0);
    }
    printf("[HEATER] STOP requested: PWM forced off\n");
}

// Return thermocouple connection state
bool heater_tc_connected(void)
{
    return g_have_tc;
}

// Get thermocouple temperature (best-effort read); returns NAN on error
float heater_get_temperature_c(void)
{
    // Return last cached temperature if available; try a fresh read otherwise.
    if (!g_initialized)
        return NAN;

    if (!isnan(g_last_tc_temp))
        return g_last_tc_temp;

    // fallback to on-demand read
    float t = read_tc_once();
    if (!isnan(t))
    {
        g_have_tc = true;
        g_last_tc_temp = t;
    }
    else
    {
        g_have_tc = false;
    }
    return t;
}

bool heater_running(void)
{
    return g_running;
}
