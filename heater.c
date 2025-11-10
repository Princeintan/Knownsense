// heater.c
#include "heater.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/spi.h"
#include <stdio.h>
#include <math.h>

#ifndef HEATER_PWM_WRAP
#define HEATER_PWM_WRAP 10000u
#endif

// PI controller constants (tune for your heater)
static const float PI_KP = 1.2f;
static const float PI_KI = 0.03f;

static mutex_t *g_spi_mutex = NULL;
static bool g_initialized = false;
static uint8_t g_pwm_pin = 2;
static uint8_t g_tc_cs_pin = 5;
static uint8_t g_control_mode = 1; // 0 = on/off, 1 = PI
static float g_target_c = 25.0f;
static float g_integral = 0.0f;
static float g_duty_percent = 0.0f;
static bool g_have_tc = false;

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

    // Read thermocouple
    float temp = read_tc_once();
    if (isnan(temp))
    {
        printf("[HEATER] TC read failed\n");
        g_have_tc = false;
        return;
    }
    g_have_tc = true;
    printf("[HEATER] read temp=%.2f target=%.2f err=%.2f integral=%.2f duty=%.2f\n",
           temp, g_target_c, g_target_c - temp, g_integral, g_duty_percent);

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
        g_duty_percent = out;
    }
    else
    {
        // on/off with hysteresis +/-1°C
        if (temp < g_target_c - 1.0f)
            g_duty_percent = 100.0f;
        else if (temp > g_target_c + 1.0f)
            g_duty_percent = 0.0f;
    }

    // Apply PWM
    uint slice = pwm_gpio_to_slice_num(g_pwm_pin);
    uint level = (uint)((g_duty_percent / 100.0f) * (float)HEATER_PWM_WRAP);
    pwm_set_chan_level(slice, pwm_gpio_to_channel(g_pwm_pin), level);
}
