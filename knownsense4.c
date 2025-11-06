#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "sd_utils.h"
#include "pico/time.h"
#include "pico/cyw43_arch.h"
#include "mqtt_client.h"
#include "lwip/stats.h"
#include "lwip/memp.h"
#include "config.h"
#include "hardware/watchdog.h"

//---------- Config ----------
static device_config_t cfg;
// ---------- Globals ----------

static uint64_t last_health_ms = 0;
extern struct stats_ lwip_stats;

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
static repeating_timer_t led_timer;

// ---------- Config / Globals ----------
MQTT_CLIENT_DATA_T mqtt;

static char log_line[256];
static char server_data[256];

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

absolute_time_t base_time;
uint32_t last_adc_read = 0;
absolute_time_t last_flush;

static bool sd_mounted = false;
static bool logging_active = false;
static sd_logger_t logger;
static char current_filename[64] = {0};

typedef struct
{
    unsigned mem_used, pbuf_used, tcpseg_used;
} leak_snap_t;
static leak_snap_t leak_base = {0};
static int health_count = 0;

static void publish_health_now(void)
{
    if (!mqtt_connected)
        return;

    uint32_t up_s = to_ms_since_boot(get_absolute_time()) / 1000u;

#if defined(LWIP_STATS) && LWIP_STATS && defined(MEM_STATS) && MEM_STATS && defined(MEMP_STATS) && MEMP_STATS
    unsigned mu = lwip_stats.mem.used;
    unsigned pu = lwip_stats.memp[MEMP_PBUF_POOL] ? lwip_stats.memp[MEMP_PBUF_POOL]->used : 0;
    unsigned tu = lwip_stats.memp[MEMP_TCP_SEG] ? lwip_stats.memp[MEMP_TCP_SEG]->used : 0;
#else
    unsigned mu = 0, pu = 0, tu = 0;
#endif

    char js[256];
    // compact JSON — easy to parse in Node-RED/Grafana/Telegraf
    snprintf(js, sizeof(js),
             "{\"up\":%lu,\"mqtt\":%d,\"inflight\":%d,"
             "\"lwip_mem\":%u,\"pbuf\":%u,\"tcpseg\":%u}",
             (unsigned long)up_s, mqtt_connected, mqtt_client_get_inflight(),
             mu, pu, tu);

    // telemetry topic: /<device>/health (retain=0)
    mqtt_client_publish_topic(&mqtt, "health", js, 0);
}

static void publish_leak_alert(unsigned mu, unsigned dmu,
                               unsigned pu, unsigned dpu,
                               unsigned tu, unsigned dtu)
{
    if (!mqtt_connected)
        return;

    char js[192];
    snprintf(js, sizeof(js),
             "{\"type\":\"leak\",\"lwip_mem\":%u,\"d_mem\":%u,"
             "\"pbuf\":%u,\"d_pbuf\":%u,\"tcpseg\":%u,\"d_tcpseg\":%u}",
             mu, dmu, pu, dpu, tu, dtu);

    // alert topic: /<device>/alert (retain=0)
    mqtt_client_publish_topic(&mqtt, "alert", js, 0);
}

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
    char base[32];
    size_t n = 0;
    for (const char *p = name; *p && n < sizeof(base) - 1; ++p)
    {
        char c = *p;
        if (c == '/' || c == '\\' || c == ':' || c == '*' || c == '?' || c == '\"' || c == '<' || c == '>' || c == '|')
            continue;
        base[n++] = c;
    }
    base[n] = '\0';
    if (base[0] == '\0')
        strncpy(base, "LOG", sizeof(base));

    snprintf(current_filename, sizeof(current_filename), "%.8s.CSV", base);

    if (sd_logger_open(&logger, current_filename) != FR_OK)
    {
        printf("[LOG] File open failed: %s\n", current_filename);
        return;
    }

    const char *csv_header = "Timestamp,ADC0_raw,Range1,Voltage1(V),Resistance1,ADC1_raw,Range2,Voltage2(V),Resistance2,Temp_raw,Temperature(C)\r\n";
    sd_logger_write(&logger, csv_header);
    sd_logger_flush(&logger);

    base_time = get_absolute_time();
    last_flush = get_absolute_time();
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
    const char *slash = strrchr(topic, '/');
    const char *verb = slash ? slash + 1 : topic;

    if (strcmp(verb, "log") == 0)
    {
        if (strcmp(payload, "0") == 0)
            stop_logging_file();
        else
            start_logging_file(payload);
    }
    else if (strcmp(verb, "cmd") == 0)
    {
        printf("[MQTT CMD] %s\n", payload);
    }
}

static inline uint32_t ms_since_boot_u32(void)
{
    return to_ms_since_boot(get_absolute_time());
}

void print_health(void)
{
    uint32_t uptime_s = ms_since_boot_u32() / 1000u;

    // Always print basic info
    printf("[HEALTH] up=%lus | mqtt_connected=%d | inflight=%d\n",
           uptime_s, mqtt_connected, mqtt_client_get_inflight());

    // ----- HEAP (lwIP MEM) -----
#if defined(LWIP_STATS) && LWIP_STATS && defined(MEM_STATS) && MEM_STATS
    extern struct stats_ lwip_stats;
    printf("         lwIP heap: used=%u free=%u max=%u err=%u\n",
           (unsigned)lwip_stats.mem.used,
           (unsigned)lwip_stats.mem.avail,
           (unsigned)lwip_stats.mem.max,
           (unsigned)lwip_stats.mem.err);
#else
    // printf("         lwIP heap: (MEM_STATS disabled)\n");
#endif

    // ----- PBUF POOL & TCP SEGMENT POOL (memp[]) -----
#if defined(LWIP_STATS) && LWIP_STATS && defined(MEMP_STATS) && MEMP_STATS
    {
        extern struct stats_ lwip_stats;

        // In your build, memp[] entries are struct stats_mem*
        const struct stats_mem *pbuf_stats = lwip_stats.memp[MEMP_PBUF_POOL];
        const struct stats_mem *tcpseg_stats = lwip_stats.memp[MEMP_TCP_SEG];

        if (pbuf_stats)
        {
            printf("         PBUF_POOL: used=%u free=%u max=%u err=%u\n",
                   (unsigned)pbuf_stats->used,
                   (unsigned)pbuf_stats->avail,
                   (unsigned)pbuf_stats->max,
                   (unsigned)pbuf_stats->err);
        }

        if (tcpseg_stats)
        {
            printf("         TCP_SEG  : used=%u free=%u max=%u err=%u\n",
                   (unsigned)tcpseg_stats->used,
                   (unsigned)tcpseg_stats->avail,
                   (unsigned)tcpseg_stats->max,
                   (unsigned)tcpseg_stats->err);
        }
    }
#else
    // printf("         memp stats: (MEMP_STATS disabled)\n");
#endif
}

// ---------- Main ----------
int main()
{
    stdio_init_all();

    sleep_ms(7000);
    watchdog_enable(8000, 1);
    printf("Knownsense starting...\n");

    // Initialize WiFi
    if (cyw43_arch_init())
        panic("WiFi init failed");

    cyw43_arch_enable_sta_mode();

    // Mount SD card once at startup
    if (sd_mount() != FR_OK)
    {
        printf("[SD] Mount failed!\n");
        while (1)
            sleep_ms(1000);
    }
    sd_mounted = true;
    // load_config_from_sd(&cfg);
    device_config_t cfg_local;
    config_init_defaults(&cfg_local);
    config_load(&cfg_local, CONFIG_PATH); // overrides with SD file if present
    if (cfg_local.sample_ms < 50)
        cfg_local.sample_ms = 50;
    if (cfg_local.sample_ms > 60000)
        cfg_local.sample_ms = 60000;
    if (cfg_local.log_flush_s == 0)
        cfg_local.log_flush_s = 1;

    memcpy(&cfg, &cfg_local, sizeof(cfg));

    watchdog_disable();

    if (cyw43_arch_wifi_connect_timeout_ms(cfg.wifi_ssid, cfg.wifi_password, CYW43_AUTH_WPA2_AES_PSK, 40000))
    {
        printf("Wi-Fi failed!\n");
    }
    else
    {
        printf("Connected to WiFi: %s\n", cfg.wifi_ssid);
        current_led_state = LED_MQTT_CONNECTING;
        update_led_behavior();
    }
    watchdog_enable(5000, 1);

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

    mqtt_client_init(&mqtt, cfg.device_name);
    mqtt.on_message = mqtt_message_handler;
    mqtt_client_start(&mqtt);

    if (mqtt_connected)
    {
        printf("Device: %s | Broker: %s | Sample: %ums | Flush: %us\n",
               cfg.device_name, cfg.broker_ip, cfg.sample_ms, cfg.log_flush_s);
        current_led_state = LED_CONNECTED;

#if defined(LWIP_STATS) && LWIP_STATS && defined(MEM_STATS) && MEM_STATS && defined(MEMP_STATS) && MEMP_STATS
        leak_base.mem_used = lwip_stats.mem.used;
        leak_base.pbuf_used = lwip_stats.memp[MEMP_PBUF_POOL] ? lwip_stats.memp[MEMP_PBUF_POOL]->used : 0;
        leak_base.tcpseg_used = lwip_stats.memp[MEMP_TCP_SEG] ? lwip_stats.memp[MEMP_TCP_SEG]->used : 0;
#endif
        publish_health_now();
    }

    else
        current_led_state = LED_MQTT_CONNECTING;
    update_led_behavior();

    base_time = get_absolute_time();
    last_flush = get_absolute_time();
    last_adc_read = time_us_32();

    while (true)
    {
        uint64_t now_ms = to_ms_since_boot(get_absolute_time());
        if (now_ms - last_health_ms > 10000)
        {
            if (stdio_usb_connected())
            {
                print_health();
            }
            if (mqtt_connected)
                publish_health_now();
            health_count++;
            if ((health_count % 6) == 0)
            { // ~1 min
#if defined(LWIP_STATS) && LWIP_STATS && defined(MEM_STATS) && MEM_STATS && defined(MEMP_STATS) && MEMP_STATS
                unsigned mu = lwip_stats.mem.used;
                unsigned pu = lwip_stats.memp[MEMP_PBUF_POOL] ? lwip_stats.memp[MEMP_PBUF_POOL]->used : 0;
                unsigned tu = lwip_stats.memp[MEMP_TCP_SEG] ? lwip_stats.memp[MEMP_TCP_SEG]->used : 0;

                if (mu > leak_base.mem_used + 1024 || pu > leak_base.pbuf_used + 2 || tu > leak_base.tcpseg_used + 2)
                {
                    unsigned dmu = mu - leak_base.mem_used;
                    unsigned dpu = pu - leak_base.pbuf_used;
                    unsigned dtu = tu - leak_base.tcpseg_used;

                    printf("[LEAK?] mem=%u(+%u) pbuf=%u(+%u) tcpseg=%u(+%u)\n",
                           mu, dmu, pu, dpu, tu, dtu);

                    publish_leak_alert(mu, dmu, pu, dpu, tu, dtu);

                    leak_base.mem_used = mu;
                    leak_base.pbuf_used = pu;
                    leak_base.tcpseg_used = tu;
                }
#endif
            }
            last_health_ms = now_ms;
        }

        if (logging_active)
        {
            read_range();

            uint32_t now = time_us_32();
            if ((now - last_adc_read) >= (cfg.sample_ms * 1000u))
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

                if (stdio_usb_connected())
                {
                    printf("%s", log_line);
                }

                sd_logger_write(&logger, log_line);
                if (absolute_time_diff_us(last_flush, get_absolute_time()) >
                    (int64_t)cfg.log_flush_s * 1000000)
                {
                    sd_logger_flush(&logger);
                    last_flush = get_absolute_time();
                }
                cyw43_arch_gpio_put(LED_PIN, 1);
                (void)cancel_repeating_timer(&write_led_timer);
                add_repeating_timer_ms(-80, write_led_off_cb, NULL, &write_led_timer);

                snprintf(server_data, sizeof(server_data),
                         "%02d:%02d:%02d,%d,%.3f,%.1f,%d,%.3f,%.1f,%.2f",
                         h, m, s,
                         current_range1, voltage_ext1, resistance1,
                         current_range2, voltage_ext2, resistance2,
                         temperature);
                if (mqtt_connected)
                    mqtt_client_publish_resistance_safe(&mqtt, server_data);

                last_adc_read = now;
            }
        }
        mqtt_client_maintenance(&mqtt);

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
        watchdog_update();

        sleep_ms(10);
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
