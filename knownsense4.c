#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "sd_utils.h"
#include "pico/time.h"
#include "pico/cyw43_arch.h"
#include "mqtt_client.h"

//---- led init and timer ----
#define LED_PIN CYW43_WL_GPIO_LED_PIN // Built-in LED on Pico W / Pico 2 W

extern volatile bool mqtt_connected; // from mqtt_client.c

typedef enum
{
    LED_BOOT,
    LED_WIFI_CONNECTING,
    LED_MQTT_CONNECTING,
    LED_CONNECTED,
    LED_LOGGING
} led_state_t;

static led_state_t current_led_state = LED_BOOT;
static led_state_t last_led_state = -1; // to detect state changes
static repeating_timer_t led_timer;

// ---------- Config / Globals ----------
MQTT_CLIENT_DATA_T mqtt;

static char log_line[128];
static char server_data[128];

#define CH1_RANGE 8
int ch1_pins[CH1_RANGE] = {0, 1, 3, 6, 7, 8, 9, 10};
int current_range1 = 3;

#define CH2_RANGE 8
int ch2_pins[CH2_RANGE] = {11, 12, 13, 14, 15, 20, 21, 22};
int current_range2 = 3;

float ch1_cf[CH1_RANGE] = {0.0278, 0.00278, 0.000278, 0.0000279, 0.00000279, 0.00000027, 0.000000027, 0.000000003};
float ch2_cf[CH2_RANGE] = {0.0364, 0.00364, 0.000364, 0.0000364, 0.00000364, 0.00000036, 0.000000036, 0.000000003};

uint16_t raw_ch1 = 1;
uint16_t raw_ch2 = 1;

#ifndef ADC_READ_INTERVAL_MS
#define ADC_READ_INTERVAL_MS 1000
#endif

absolute_time_t base_time;
uint32_t last_adc_read = 0;
// absolute_time_t last_flush;

static bool sd_mounted = false;
static bool logging_active = false;
static sd_logger_t logger;
static char current_filename[64] = {0};

// ---------- Helpers ----------
void range_selector1(void)
{
    for (int i = 0; i < CH1_RANGE; i++)
        gpio_put(ch1_pins[i], 0);
    gpio_put(ch1_pins[current_range1 - 1], 1);
}
void range_selector2(void)
{
    for (int i = 0; i < CH2_RANGE; i++)
        gpio_put(ch2_pins[i], 0);
    gpio_put(ch2_pins[current_range2 - 1], 1);
}

void read_range(void)
{
    adc_select_input(0);
    raw_ch1 = adc_read();
    adc_select_input(1);
    raw_ch2 = adc_read();

    if (raw_ch1 > 3950)
        current_range1++;
    if (raw_ch1 < 210)
        current_range1--;
    if (current_range1 < 1)
        current_range1 = 1;
    if (current_range1 > CH1_RANGE)
        current_range1 = CH1_RANGE;
    range_selector1();

    if (raw_ch2 > 3950)
        current_range2++;
    if (raw_ch2 < 210)
        current_range2--;
    if (current_range2 < 1)
        current_range2 = 1;
    if (current_range2 > CH2_RANGE)
        current_range2 = CH2_RANGE;
    range_selector2();
}

// led blink functions -----
bool led_blink_callback(repeating_timer_t *t)
{
    static bool led_on = false;

    switch (current_led_state)
    {
    case LED_BOOT:
    case LED_WIFI_CONNECTING:
    case LED_MQTT_CONNECTING:
        led_on = !led_on;
        cyw43_arch_gpio_put(LED_PIN, led_on);
        break;

    case LED_CONNECTED:
        cyw43_arch_gpio_put(LED_PIN, 1); // solid ON
        break;

    case LED_LOGGING:
        // handled manually during write
        break;
    }
    return true;
}

void update_led_behavior()
{
    cancel_repeating_timer(&led_timer);
    int interval_ms = 500;

    switch (current_led_state)
    {
    case LED_BOOT:
        interval_ms = 200;
        add_repeating_timer_ms(-interval_ms, led_blink_callback, NULL, &led_timer);
        break;

    case LED_WIFI_CONNECTING:
        interval_ms = 200;
        add_repeating_timer_ms(-interval_ms, led_blink_callback, NULL, &led_timer);
        break;

    case LED_MQTT_CONNECTING:
        interval_ms = 400;
        add_repeating_timer_ms(-interval_ms, led_blink_callback, NULL, &led_timer);
        break;

    case LED_CONNECTED:
        cyw43_arch_gpio_put(LED_PIN, 1); // solid ON (ready to log)
        break;

    case LED_LOGGING:
        cyw43_arch_gpio_put(LED_PIN, 0); // normally OFF
        break;
    }
}

static repeating_timer_t write_led_timer;
bool write_led_off_cb(repeating_timer_t *t)
{
    cyw43_arch_gpio_put(LED_PIN, 0);
    return false; // one-shot
}

// ---------- Logging control ----------

static void start_logging_file(const char *name)
{
    if (!sd_mounted)
    {
        printf("[SD] SD card not mounted! Cannot start logging.\n");
        return;
    }

    if (logging_active)
    {
        printf("[LOG] Already logging to %s\n", current_filename);
        return;
    }

    snprintf(current_filename, sizeof(current_filename), "%s.csv", name);

    if (sd_logger_open(&logger, current_filename) != FR_OK)
    {
        printf("[LOG] File open failed: %s\n", current_filename);
        return;
    }

    const char *csv_header = "Timestamp,ADC0_raw,Range1,Voltage1(V),Resistance1,ADC1_raw,Range2,Voltage2(V),Resistance2,Temp_raw,Temperature(C)\r\n";
    sd_logger_write(&logger, csv_header);
    sd_logger_flush(&logger);

    base_time = get_absolute_time();
    // last_flush = get_absolute_time();
    last_adc_read = time_us_32();
    logging_active = true;

    printf("[LOG] Started logging to %s\n", current_filename);
    current_led_state = LED_LOGGING;
    update_led_behavior();
}

static void stop_logging_file(void)
{
    if (!logging_active)
        return;

    sd_logger_flush(&logger);
    sd_logger_close(&logger);
    logging_active = false;
    current_filename[0] = '\0';
    printf("[LOG] Logging stopped and file closed.\n");
    current_led_state = (mqtt_connected ? LED_CONNECTED : LED_MQTT_CONNECTING);
    update_led_behavior();
}

// ---------- MQTT message handler ----------
void mqtt_message_handler(const char *topic, const char *payload)
{
    printf("Received MQTT message on topic: %s payload: %s\n", topic, payload);

    if (strcmp(topic, "log") == 0)
    {
        if (strcmp(payload, "0") == 0)
        {
            stop_logging_file();
        }
        else
        {
            start_logging_file(payload);
        }
    }
    else if (strcmp(topic, "cmd") == 0)
    {
        printf("[MQTT CMD] %s\n", payload);
    }
}

// ---------- Main ----------
int main()
{
    stdio_init_all();
    // pico_set_program_stack_limit(0x2000);
    // watchdog_enable(10000, 1); // 10 s watchdog

    sleep_ms(7000);
    printf("Knownsense [K01] starting...\n");
    // cyw43_arch_gpio_put(LED_PIN, 0);
    // current_led_state = LED_BOOT;
    // update_led_behavior();

    // Initialize WiFi
    if (cyw43_arch_init())
        panic("WiFi init failed");

    cyw43_arch_enable_sta_mode();

    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000))
    {
        printf("Wi-Fi failed!\n");
    }
    else
    {
        printf("Connected to WiFi: %s\n", WIFI_SSID);
        current_led_state = LED_MQTT_CONNECTING;
        update_led_behavior();
    }

    // Mount SD card once at startup
    printf("[SD] Mounting SD card...\n");
    if (sd_mount() != FR_OK)
    {
        printf("[SD] Mount failed!\n");
        while (1)
            sleep_ms(1000);
    }
    sd_mounted = true;
    printf("[SD] SD card mounted.\n");

    // Initialize GPIO pins
    for (int i = 0; i < CH1_RANGE; i++)
    {
        gpio_init(ch1_pins[i]);
        gpio_set_dir(ch1_pins[i], true);
        gpio_put(ch1_pins[i], 0);
    }
    for (int i = 0; i < CH2_RANGE; i++)
    {
        gpio_init(ch2_pins[i]);
        gpio_set_dir(ch2_pins[i], true);
        gpio_put(ch2_pins[i], 0);
    }
    sleep_ms(20);
    gpio_put(ch1_pins[current_range1 - 1], 1);
    gpio_put(ch2_pins[current_range2 - 1], 1);

    // ADC init
    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_gpio_init(26);
    adc_gpio_init(27);

    // MQTT init

    mqtt_client_init(&mqtt, "K01");
    mqtt.on_message = mqtt_message_handler;
    mqtt_client_start(&mqtt);

    if (mqtt_connected)
        current_led_state = LED_CONNECTED;
    else
        current_led_state = LED_MQTT_CONNECTING;
    update_led_behavior();

    char buffer[3] = {0};
    int buf_idx = 0;

    base_time = get_absolute_time();
    // last_flush = get_absolute_time();
    last_adc_read = time_us_32();

    while (true)
    {
        if (logging_active)
        {
            read_range();

            uint32_t now = time_us_32();
            if ((now - last_adc_read) >= (ADC_READ_INTERVAL_MS * 1000))
            {
                const float VREF = 3.3f;
                float voltage_ext1 = (raw_ch1 * VREF) / (1 << 12);
                float voltage_ext2 = (raw_ch2 * VREF) / (1 << 12);
                float resistance1 = voltage_ext1 / ch1_cf[current_range1 - 1];
                float resistance2 = voltage_ext2 / ch2_cf[current_range2 - 1];

                adc_select_input(4);
                uint16_t raw_temp = adc_read();
                float voltage_temp = (raw_temp * VREF) / (1 << 12);
                float temperature = 27.0f - (voltage_temp - 0.706f) / 0.001721f;

                int64_t elapsed_us = absolute_time_diff_us(base_time, get_absolute_time());
                int h = (elapsed_us / 1000000) / 3600;
                int m = ((elapsed_us / 1000000) % 3600) / 60;
                int s = (elapsed_us / 1000000) % 60;

                // char log_line[256];
                snprintf(log_line, sizeof(log_line),
                         "%02d:%02d:%02d,%u,%d,%.3f,%.1f,%u,%d,%.3f,%.1f,%u,%.2f\r\n",
                         h, m, s,
                         raw_ch1, current_range1, voltage_ext1, resistance1,
                         raw_ch2, current_range2, voltage_ext2, resistance2,
                         raw_temp, temperature);

                printf("%s", log_line);

                sd_logger_write(&logger, log_line);
                // brief ON blink when data is written
                // non-blocking: set LED on then schedule it off in 80 ms
                cyw43_arch_gpio_put(LED_PIN, 1);
                cancel_repeating_timer(&write_led_timer);
                add_repeating_timer_ms(-80, write_led_off_cb, NULL, &write_led_timer);
                sd_logger_flush(&logger);

                // char server_data[128];
                snprintf(server_data, sizeof(server_data),
                         "%02d:%02d:%02d,%d,%.3f,%.1f,%d,%.3f,%.1f,%.2f",
                         h, m, s,
                         current_range1, voltage_ext1, resistance1,
                         current_range2, voltage_ext2, resistance2,
                         temperature);
                if (mqtt_connected)
                    mqtt_client_publish_resistance_safe(&mqtt, server_data);
                // mqtt_client_publish_resistance(&mqtt, server_data);

                // static absolute_time_t last_maint = 0;
                // if (absolute_time_diff_us(last_maint, get_absolute_time()) > 1000000)
                // {
                //     sys_check_timeouts();
                //     last_maint = get_absolute_time();
                // }

                mqtt_client_maintenance(&mqtt);
                //     static absolute_time_t last_maintenance = 0;

                // if (absolute_time_diff_us(last_maintenance, get_absolute_time()) > 10 * 1000000)
                // {
                //     mqtt_client_maintenance(&mqtt);
                //     last_maintenance = get_absolute_time();
                // }

                //-----------------------------------

                // if (absolute_time_diff_us(last_flush, get_absolute_time()) > 1000000)
                // {
                //     sd_logger_flush(&logger);
                //     last_flush = get_absolute_time();
                //     printf("[SD] Data flushed.\n");
                // }

                last_adc_read = now;
            }
        }

        // Serial input non-blocking manual range override
        int c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT)
        {
            if ((c >= '0' && c <= '9') && buf_idx < 2)
            {
                buffer[buf_idx++] = (char)c;
            }
            else if (c == '\n' && buf_idx > 0)
            {
                buffer[buf_idx] = '\0';
                int ch = buffer[0] - '0';
                int range = (buf_idx > 1) ? (buffer[1] - '0') : -1;

                if (ch == 1 && range >= 1 && range <= CH1_RANGE)
                {
                    for (int i = 0; i < CH1_RANGE; i++)
                        gpio_put(ch1_pins[i], 0);
                    gpio_put(ch1_pins[range - 1], 1);
                    current_range1 = range;
                    printf("Channel 1 -> GPIO %d HIGH\n", ch1_pins[range - 1]);
                }
                else if (ch == 2 && range >= 1 && range <= CH2_RANGE)
                {
                    for (int i = 0; i < CH2_RANGE; i++)
                        gpio_put(ch2_pins[i], 0);
                    gpio_put(ch2_pins[range - 1], 1);
                    current_range2 = range;
                    printf("Channel 2 -> GPIO %d HIGH\n", ch2_pins[range - 1]);
                }
                else if (ch == 0)
                {
                    for (int i = 0; i < CH1_RANGE; i++)
                        gpio_put(ch1_pins[i], 0);
                    for (int i = 0; i < CH2_RANGE; i++)
                        gpio_put(ch2_pins[i], 0);
                    printf("All GPIOs LOW\n");
                }
                buf_idx = 0;
                memset(buffer, 0, sizeof(buffer));
            }
            else
            {
                buf_idx = 0;
                memset(buffer, 0, sizeof(buffer));
            }
        }

        // --- LED state monitor ---
        led_state_t new_state = current_led_state;
        if (logging_active)
            new_state = LED_LOGGING;
        else if (mqtt_connected)
            new_state = LED_CONNECTED;
        else
            new_state = LED_MQTT_CONNECTING;

        if (new_state != current_led_state)
        {
            current_led_state = new_state;
            update_led_behavior();
        }
        // watchdog_update();
        // tight_loop_contents();

        sleep_ms(5);
    }

    // cleanup (never reached normally)
    if (logging_active)
    {
        sd_logger_flush(&logger);
        sd_logger_close(&logger);
    }
    mqtt_client_stop(&mqtt);
    return 0;
}
