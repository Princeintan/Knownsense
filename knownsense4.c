

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "sd_utils.h"
#include "pico/time.h"
#include "pico/cyw43_arch.h"
#include "mqtt_client.h"

MQTT_CLIENT_DATA_T mqtt;

#define ch1_range 8
int ch1_pins[ch1_range] = {0, 1, 3, 6, 7, 8, 9, 10};
int current_range1 = 3;
#define ch2_range 8
int ch2_pins[ch2_range] = {11, 12, 13, 14, 15, 20, 21, 22};
int current_range2 = 3;
int spi_pins[] = {16, 17, 18, 19};
#define NUM_SPI_PINS (sizeof(spi_pins) / sizeof(spi_pins[0]))

float ch1_cf[ch1_range] = {0.0278, 0.00278, 0.000278, 0.0000279, 0.00000279, 0.00000027, 0.000000027, 0.000000003};
float ch2_cf[ch2_range] = {0.0364, 0.00364, 0.000364, 0.0000364, 0.00000364, 0.00000036, 0.000000036, 0.000000003};
int channel_selected = 0;

uint16_t raw_ch1 = 1;
uint16_t raw_ch2 = 1;

#ifndef ADC_READ_INTERVAL_MS
#define ADC_READ_INTERVAL_MS 1000 // ADC reading and printing interval (1 second)
#endif

absolute_time_t base_time;

void range_selector1()
{
    for (int i = 0; i < ch1_range; i++)
    {
        gpio_put(ch1_pins[i], 0);
    }
    gpio_put(ch1_pins[current_range1 - 1], 1);
}
void range_selector2()
{
    for (int i = 0; i < ch2_range; i++)
    {
        gpio_put(ch2_pins[i], 0);
    }
    gpio_put(ch2_pins[current_range2 - 1], 1);
}

void read_range()
{
    adc_select_input(0); // Channel 0 = GPIO26
    raw_ch1 = adc_read();
    adc_select_input(1); // Channel 1 = GPIO27
    raw_ch2 = adc_read();
    if (raw_ch1 > 3950)
    {
        current_range1++;
        range_selector1();
    }
    if (raw_ch2 > 3950)
    {
        current_range2++;
        range_selector2();
    }
    if (raw_ch1 < 210)
    {
        current_range1--;
        range_selector1();
    }
    if (raw_ch2 < 210)
    {
        current_range2--;
        range_selector2();
    }

    if (current_range1 < 1)
        current_range1 = 1;
    if (current_range1 > 8)
        current_range1 = 8;

    if (current_range2 < 1)
        current_range2 = 1;
    if (current_range2 > 8)
        current_range2 = 8;
}

void mqtt_message_handler(const char *topic, const char *payload)
{
    if (strcmp(topic, "log") == 0)
    {
        printf("Received LED command: %s\n", payload);
    }
    else if (strcmp(topic, "cmd") == 0)
    {
        printf("Received Command: %s\n", payload);
        // you could parse and display or store
    }
    else
    {
        printf("Unhandled topic: %s, message: %s\n", topic, payload);
    }
}

int main()
{
    stdio_init_all();
    sleep_ms(8000); // Wait for USB serial to stabilize
    printf("Knownsense3 starting up...\n");

    if (cyw43_arch_init())
        panic("WiFi init failed");
    cyw43_arch_enable_sta_mode();

    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000))
        panic("WiFi connect failed");

    printf("Connected to WiFi: %s\n", WIFI_SSID);

    // Ask for filename over serial
    char file_str[32];
    int idx = 0;
    printf("Enter timestamp string (HHMMSS):\n");
    while (1)
    {
        int c = getchar_timeout_us(100000);
        if (c != PICO_ERROR_TIMEOUT)
        {
            if (c == '\n')
            {
                file_str[idx] = '\0';
                break;
            }
            if (idx < 31)
                file_str[idx++] = (char)c;
        }
    }

    // Mount SD
    if (sd_mount() != FR_OK)
    {
        printf("SD mount failed!\n");
        while (1)
            sleep_ms(1000);
    }

    // Build filename directly in root
    char filename[64];
    snprintf(filename, sizeof(filename), "%s.csv", file_str);

    // Open file
    sd_logger_t logger;
    if (sd_logger_open(&logger, filename) != FR_OK)
    {
        printf("File open failed!\n");
        while (1)
            sleep_ms(1000);
    }

    // --- Write CSV headers once ---
    const char *csv_header = "Timestamp,ADC0_raw,Range1,Voltage1(V),Resistance1,ADC1_raw,Range2,Voltage2(V),Resistance2,Temp_raw,Temperature(C)\r\n";
    sd_logger_write(&logger, csv_header);

    printf("Logging to: %s\n", filename);

    uint32_t counter = 0;
    absolute_time_t last_flush = get_absolute_time();
    // Initialize ch1_pins as outputs
    for (int i = 0; i < ch1_range; i++)
    {
        gpio_init(ch1_pins[i]);
        gpio_set_dir(ch1_pins[i], true);
        gpio_put(ch1_pins[i], 0); // Start LOW
    }
    for (int i = 0; i < ch2_range; i++)
    {
        gpio_init(ch2_pins[i]);
        gpio_set_dir(ch2_pins[i], true);
        gpio_put(ch2_pins[i], 0); // Start LOW
    }
    sleep_ms(20);
    gpio_put(ch1_pins[current_range1 - 1], 1);
    gpio_put(ch2_pins[current_range2 - 1], 1);

    // Initialize ADC
    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_gpio_init(26); // GPIO26 (ADC0)
    adc_gpio_init(27); // GPIO27 (ADC1)

    printf("Send 1-%d over serial to enable corresponding GPIO\n", ch1_range);
    uint32_t last_adc_read = time_us_32(); // Timestamp for ADC reading
    char buffer[3] = {0};                  // Buffer for up to 2 digits + null terminator
    int buf_idx = 0;

    base_time = get_absolute_time();

    mqtt_client_init(&mqtt, "K01");

    mqtt.on_message = mqtt_message_handler;
    mqtt_client_start(&mqtt);

    while (true)
    {

        read_range();
        // Non-blocking ADC reading and printing
        uint32_t current_time = time_us_32();
        if ((current_time - last_adc_read) >= (ADC_READ_INTERVAL_MS * 1000))
        {

            // Convert to voltage (assuming 3.3V VREF)
            const float VREF = 3.3f;
            float voltage_ext1 = (raw_ch1 * VREF) / (1 << 12);
            float voltage_ext2 = (raw_ch2 * VREF) / (1 << 12);

            float resistance1 = voltage_ext1 / ch1_cf[current_range1 - 1];
            float resistance2 = voltage_ext2 / ch2_cf[current_range2 - 1];

            // Read internal temperature sensor (ADC4)
            adc_select_input(4); // Channel 4 = temperature sensor
            uint16_t raw_temp = adc_read();

            // Convert to voltage
            float voltage_temp = (raw_temp * VREF) / (1 << 12);

            // Convert voltage to temperature (approx formula from datasheet)
            float temperature = 27.0f - (voltage_temp - 0.706f) / 0.001721f;

            // Compute timestamp relative to base
            int64_t elapsed_us = absolute_time_diff_us(base_time, get_absolute_time());
            int h = (elapsed_us / 1000000) / 3600;
            int m = ((elapsed_us / 1000000) % 3600) / 60;
            int s = (elapsed_us / 1000000) % 60;

            char log_line[256];
            snprintf(log_line, sizeof(log_line),
                     "%02d:%02d:%02d,%u,%d,%.3f,%.1f,%u,%d,%.3f,%.1f,%u,%.2f\r\n",
                     h, m, s, raw_ch1, current_range1, voltage_ext1, resistance1,
                     raw_ch2, current_range2, voltage_ext2, resistance2,
                     raw_temp, temperature);

            printf("%s", log_line);
            sd_logger_write(&logger, log_line);

            char server_data[128];
            snprintf(server_data, sizeof(server_data),
                     "%02d:%02d:%02d,%d,%.3f,%.1f,%d,%.3f,%.1f,%.2f\r\n",
                     h, m, s,
                     current_range1, voltage_ext1, resistance1,
                     current_range2, voltage_ext2, resistance2,
                     temperature);

            mqtt_client_publish_resistance(&mqtt, server_data);

            if (absolute_time_diff_us(last_flush, get_absolute_time()) > 2000000)
            {
                sd_logger_flush(&logger);
                printf("[SD] Data flushed.\n");
                last_flush = get_absolute_time();
            }

            last_adc_read = current_time;
        }

        // Non-blocking serial input handling
        int c = getchar_timeout_us(0); // Non-blocking read
        if (c != PICO_ERROR_TIMEOUT)
        {
            if ((c >= '0' && c <= '9') && buf_idx < 2)
            {
                buffer[buf_idx++] = (char)c;
            }
            else if (c == '\n' && buf_idx > 0)
            {
                buffer[buf_idx] = '\0';             // Null-terminate
                int range = buffer[1] - '0';        // First digit
                channel_selected = buffer[0] - '0'; // Second digit if exists

                printf("Received number: %d, selected channel : %d\n", range, channel_selected);
                if (range >= 1 && range <= ch1_range)
                {
                    if (channel_selected == 1)
                    {
                        // Turn all ch1_pins LOW
                        for (int i = 0; i < ch1_range; i++)
                        {
                            gpio_put(ch1_pins[i], 0);
                        }
                        // Turn selected pin HIGH

                        gpio_put(ch1_pins[range - 1], 1);
                        current_range1 = range;
                        printf("at channel 1 GPIO %d HIGH, others LOW\n", ch1_pins[range - 1]);
                    }
                    else if (channel_selected == 2)
                    {
                        for (int i = 0; i < ch2_range; i++)
                        {
                            gpio_put(ch2_pins[i], 0);
                        }
                        // Turn selected pin HIGH
                        gpio_put(ch2_pins[range - 1], 1);
                        current_range2 = range;
                        printf("at cahnnel 2 GPIO %d HIGH, others LOW\n", ch2_pins[range - 1]);
                    }
                    else
                    {
                        printf("Invalid channel selected: %d, choose 1 or 2\n", channel_selected);
                    }
                }
                else if (range == 0)
                {
                    // Turn all ch1_pins LOW
                    for (int i = 0; i < ch1_range; i++)
                    {
                        gpio_put(ch1_pins[i], 0);
                    }
                    for (int i = 0; i < ch2_range; i++)
                    {
                        gpio_put(ch2_pins[i], 0);
                    }
                    printf("All GPIOs LOW\n");
                }
                else if (range > ch1_range)
                {
                    printf("Invalid input pin number: %s\n, send digits smaller than 8\n", buffer);
                }
                else
                {
                    printf("Invalid input: %s\n, send digits followed by newline\n", buffer);
                }
                buf_idx = 0; // Reset for next input
                memset(buffer, 0, sizeof(buffer));
            }
            else
            {
                buf_idx = 0; // Reset on invalid input
                memset(buffer, 0, sizeof(buffer));
                if (c != '\n')
                    printf("Invalid input: %c, send digits followed by newline\n", c);
            }
        }
    }
    // sd_logger_force_flush(&logger);
    sd_logger_close(&logger);
}