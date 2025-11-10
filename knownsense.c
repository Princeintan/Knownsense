#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "pico/mutex.h"
#include "sd_utils.h"
#include "pico/time.h"
#include "pico/cyw43_arch.h"
#include "mqtt_client.h"
#include "lwip/stats.h"
#include "lwip/memp.h"
#include "config.h"
#include "heater.h"
#include "hardware/watchdog.h"

//---------- Config ----------
static device_config_t cfg;
static mutex_t spi_mutex;
// ---------- Globals ----------

static uint64_t last_health_ms = 0;
extern struct stats_ lwip_stats;
static uint32_t day_count = 0; // cumulative days since user started logging
static uint32_t sod_start = 0; // seconds-of-day at (this) boot start

// ---- Policy knobs (no dynamic memory anywhere) ----
#define LOG_ROLL_ROWS 10000u        // lines/file
#define NET_RETRY_WINDOW_MS 120000u // reboot if MQTT down this long

// small per-boot jitter (0..999 ms) to spread reboot timing
static inline uint32_t reboot_jitter_ms(void)
{
    return (uint32_t)(time_us_32() % 1000u);
}

#define WIFI_RETRY_PERIOD_MS 10000u  // Wi-Fi retry cadence
#define AUTOLOG_FILE_PATH "AUTO.TXT" // must satisfy 5-char + .TXT/.CSV rule

static uint32_t log_line_count = 0;
static uint8_t file_index = 1; // 01..99
static char base3[4] = "LOG";  // exactly 3 chars + NUL
static char ext3[4] = "CSV";   // "CSV" or "TXT" (fixed)

static uint64_t last_mqtt_ok_ms = 0;
static uint64_t last_wifi_retry_ms = 0;
static uint64_t last_heater_ms = 0;
const uint32_t HEATER_PERIOD_MS = 500;

static inline void sanitize_base3_from_payload(const char *p)
{
    // take first 3 alnum, uppercase
    int k = 0;
    for (; *p && k < 3; ++p)
    {
        char c = *p;
        if ((c >= 'a' && c <= 'z'))
            c -= 32;
        if ((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9'))
            base3[k++] = c;
    }
    while (k < 3)
        base3[k++] = 'L'; // pad if too short
    base3[3] = '\0';
}

// ABC01.CSV — always 12 bytes incl NUL
static inline void build_filename(char out[13])
{
    // base3 + 2 digits
    out[0] = base3[0];
    out[1] = base3[1];
    out[2] = base3[2];
    out[3] = (char)('0' + (file_index / 10));
    out[4] = (char)('0' + (file_index % 10));
    out[5] = '.';
    out[6] = ext3[0];
    out[7] = ext3[1];
    out[8] = ext3[2];
    out[9] = '\0';
}

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
    mqtt_client_publish_topic(&mqtt, "health", js, 1);
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
    mqtt_client_publish_topic(&mqtt, "alert", js, 1);
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
static bool save_autolog_marker(void)
{
    FIL f;
    if (f_open(&f, AUTOLOG_FILE_PATH, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK)
        return false;
    char buf[64];
    // base=ABC\nidx=01\next=CSV\nday=123\nsod=456\n
    UINT n;
    int len = snprintf(buf, sizeof(buf),
                       "base=%c%c%c\nidx=%02u\next=%c%c%c\nday=%u\nsod=%u\n",
                       base3[0], base3[1], base3[2], (unsigned)file_index,
                       ext3[0], ext3[1], ext3[2],
                       (unsigned)day_count, (unsigned)sod_start);
    FRESULT fr = f_write(&f, buf, (UINT)len, &n);
    f_close(&f);
    return (fr == FR_OK && n == (UINT)len);
}

static void clear_autolog_marker(void) { f_unlink(AUTOLOG_FILE_PATH); }

static bool load_autolog_marker(void)
{
    FIL f;
    if (f_open(&f, AUTOLOG_FILE_PATH, FA_READ) != FR_OK)
        return false;

    char line[32];
    char b0 = 'L', b1 = 'O', b2 = 'G';
    unsigned idx = 1, dtmp = 0, sod = 0;
    char e0 = 'C', e1 = 'S', e2 = 'V';

    while (f_gets(line, sizeof(line), &f))
    {
        if (sscanf(line, "base=%c%c%c", &b0, &b1, &b2) == 3)
        {
        }
        else if (sscanf(line, "idx=%u", &idx) == 1)
        {
        }
        else if (sscanf(line, "ext=%c%c%c", &e0, &e1, &e2) == 3)
        {
        }
        else if (sscanf(line, "day=%u", &dtmp) == 1)
        {
        }
        else if (sscanf(line, "sod=%u", &sod) == 1)
        {
        }
    }
    f_close(&f);

    if (idx == 0 || idx > 99)
        idx = 1;

    base3[0] = b0;
    base3[1] = b1;
    base3[2] = b2;
    base3[3] = '\0';
    ext3[0] = e0;
    ext3[1] = e1;
    ext3[2] = e2;
    ext3[3] = '\0';
    file_index = (uint8_t)idx;
    day_count = dtmp; // persist days across reboot
    sod_start = sod;  // seconds-of-day continuity across reboot
    return true;
}

// watch dog -----
#define WD_MAGIC 0x4B4E4F57u
static inline void wd_set(int i, uint32_t v) { watchdog_hw->scratch[i] = v; }
static inline uint32_t wd_get(int i) { return watchdog_hw->scratch[i]; }
static void wd_store_state(bool was_logging, const char *curr_fn)
{
    wd_set(0, WD_MAGIC);
    wd_set(1, (was_logging ? 1u : 0u) | ((uint32_t)file_index << 8));
    wd_set(2, (uint32_t)log_line_count);
    // pack curr_fn (<= 9 bytes incl NUL; we store 9 safely in 3 regs)
    char fn[9] = {0};
    for (int i = 0; i < 8 && curr_fn[i]; ++i)
        fn[i] = curr_fn[i]; // "ABC01.CSV"
    wd_set(3, (uint32_t)fn[0] | ((uint32_t)fn[1] << 8) | ((uint32_t)fn[2] << 16) | ((uint32_t)fn[3] << 24));
    wd_set(4, (uint32_t)fn[4] | ((uint32_t)fn[5] << 8) | ((uint32_t)fn[6] << 16) | ((uint32_t)fn[7] << 24));
    wd_set(5, (uint32_t)fn[8]);
    // base3 + ext3 for completeness
    wd_set(6, (uint32_t)base3[0] | ((uint32_t)base3[1] << 8) | ((uint32_t)base3[2] << 16) | ((uint32_t)ext3[0] << 24));
    wd_set(7, (uint32_t)ext3[1] | ((uint32_t)ext3[2] << 8));
}
static void wd_clear_state(void)
{
    for (int i = 0; i < 8; ++i)
        wd_set(i, 0);
}
static bool wd_load_state(bool *was_logging, char *fn_out /*>=10*/)
{
    if (wd_get(0) != WD_MAGIC)
        return false;
    uint32_t f1 = wd_get(1);
    *was_logging = (f1 & 1u) != 0;
    file_index = (uint8_t)((f1 >> 8) & 0xFF);
    log_line_count = wd_get(2);
    uint32_t w3 = wd_get(3), w4 = wd_get(4), w5 = wd_get(5), w6 = wd_get(6), w7 = wd_get(7);
    fn_out[0] = w3 & 0xFF;
    fn_out[1] = (w3 >> 8) & 0xFF;
    fn_out[2] = (w3 >> 16) & 0xFF;
    fn_out[3] = (w3 >> 24) & 0xFF;
    fn_out[4] = w4 & 0xFF;
    fn_out[5] = (w4 >> 8) & 0xFF;
    fn_out[6] = (w4 >> 16) & 0xFF;
    fn_out[7] = (w4 >> 24) & 0xFF;
    fn_out[8] = w5 & 0xFF;
    fn_out[9] = '\0';
    base3[0] = w6 & 0xFF;
    base3[1] = (w6 >> 8) & 0xFF;
    base3[2] = (w6 >> 16) & 0xFF;
    ext3[0] = (w6 >> 24) & 0xFF;
    ext3[1] = w7 & 0xFF;
    ext3[2] = (w7 >> 8) & 0xFF;
    base3[3] = ext3[3] = '\0';
    wd_clear_state();
    return true;
}

static bool open_new_log_file(void)
{
    build_filename(current_filename);
    FRESULT fr = sd_logger_open(&logger, current_filename); // create new file handle
    if (fr != FR_OK)
    {
        printf("[LOG] open failed: %s fr=%d\n", current_filename, fr);
        return false;
    }
    static const char *csv_header =
        "Day,Timestamp,ADC0_raw,Range1,Voltage1(V),Resistance1,ADC1_raw,Range2,Voltage2(V),Resistance2,Temp_raw,Temperature(C)\r\n";

    sd_logger_write(&logger, csv_header);
    sd_logger_flush(&logger);
    log_line_count = 0;
    wd_store_state(true, current_filename);
    printf("[LOG] Started %s\n", current_filename);
    return true;
}
static inline void close_current_log_file(void)
{
    sd_logger_flush(&logger);
    sd_logger_close(&logger);
}

static void start_logging_file(const char *payload_base)
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

    // optional sanitize from payload into base3 (3 chars)
    if (payload_base && *payload_base)
        sanitize_base3_from_payload(payload_base);

    // choose the next free index (01..99)
    for (uint8_t tries = 0; tries < 99; ++tries)
    {
        char cand[13];
        build_filename(cand);
        FILINFO fi;
        if (f_stat(cand, &fi) != FR_OK)
            break; // free slot
        if (file_index < 99)
            ++file_index;
        else
            file_index = 1;
    }
    if (!open_new_log_file())
        return;

    base_time = get_absolute_time();
    last_flush = get_absolute_time();
    last_adc_read = time_us_32();
    logging_active = true;

    current_led_state = LED_LOGGING;
    update_led_behavior();

    (void)save_autolog_marker(); // persist intent so we auto-resume
}

static void stop_logging_file(void)
{
    if (!logging_active)
        return;

    close_current_log_file();
    logging_active = false;
    current_filename[0] = '\0';
    wd_clear_state();
    clear_autolog_marker(); // explicit stop cancels autostart

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
        {
            stop_logging_file();
        }
        else
        {
            if (strstr(payload, ":TXT"))
            {
                ext3[0] = 'T';
                ext3[1] = 'X';
                ext3[2] = 'T';
            }
            else
            {
                ext3[0] = 'C';
                ext3[1] = 'S';
                ext3[2] = 'V';
            }

            // Sanitize base (first 3 A-Z0-9) inside start_logging_file()
            start_logging_file(payload);
        }
    }
    else if (strcmp(verb, "set") == 0)
    {
        float v = strtof(payload, NULL);
        if (!isnan(v))
        {
            heater_set_target_c(v);
            printf("[MQTT SET] heater target set to %.2f C via MQTT\n", v);
        }
        else
        {
            printf("[MQTT SET] unrecognized payload: '%s'\n", payload);
        }
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

    // initialize SPI mutex and register with heater module
    mutex_init(&spi_mutex);
    heater_register_spi_mutex(&spi_mutex);

    // initialize heater if enabled by configuration
    if (cfg.heater_enabled)
    {
        heater_init_if_enabled(cfg.heater_pwm_pin, cfg.heater_tc_cs_pin, cfg.heater_control_mode, cfg.heater_target_c);
    }

    watchdog_disable();

    // If marker exists do autostart. WD-only (without marker) means user had stopped logging, so do NOT auto-resume.

    bool wd_was_logging = false;
    char wd_fn[10] = {0};
    bool have_wd = wd_load_state(&wd_was_logging, wd_fn) && wd_was_logging;
    bool have_marker = load_autolog_marker(); // loads base3/ext3 & file_index

    if (have_marker)
    {
        // To avoid appending partial lines after a WD reset: start a NEW rolled file
        if (have_wd)
        {
            if (file_index < 99)
                ++file_index;
            else
                file_index = 1;
        }
        start_logging_file(NULL); // base3/ext3/index already set
    }

    // Network health baseline
    last_mqtt_ok_ms = to_ms_since_boot(get_absolute_time());
    last_wifi_retry_ms = last_mqtt_ok_ms; // start the retry window from now

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
        if (now_ms - last_health_ms > 30000)
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

                    if (stdio_usb_connected())
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
                // -- formating elapsed time --
                int64_t elapsed_us = absolute_time_diff_us(base_time, get_absolute_time());
                uint32_t elapsed_s = (uint32_t)(elapsed_us / 1000000u);

                // Add elapsed to persisted start-of-day
                uint32_t total_s = sod_start + elapsed_s;
                uint32_t days_add = total_s / 86400u;
                uint32_t sod = total_s % 86400u;

                // If we crossed a day boundary since last loop, bump day_count and reduce sod_start
                if (days_add > 0)
                {
                    day_count += days_add;
                    // Move the reference so elapsed stays small and precise
                    sod_start = sod;                 // carry remainder as new start-of-day
                    base_time = get_absolute_time(); // reset base for small elapsed
                }

                // Format h:m:s from 'sod'
                int h = (int)(sod / 3600u);
                int m = (int)((sod % 3600u) / 60u);
                int s = (int)(sod % 60u);

                // --- formating elevated time done ---

                snprintf(log_line, sizeof(log_line),
                         "%u,%02d:%02d:%02d,%u,%d,%.3f,%.1f,%u,%d,%.3f,%.1f,%u,%.2f\r\n",
                         (unsigned)day_count, h, m, s,
                         raw_ch1, current_range1, voltage_ext1, resistance1,
                         raw_ch2, current_range2, voltage_ext2, resistance2,
                         raw_temp, temperature);

                if (stdio_usb_connected())
                {
                    printf("%s", log_line);
                }

                sd_logger_write(&logger, log_line);
                // COUNT + PERSIST ON FLUSH
                log_line_count++;

                if (absolute_time_diff_us(last_flush, get_absolute_time()) >
                    (int64_t)cfg.log_flush_s * 1000000)
                {
                    sd_logger_flush(&logger);
                    last_flush = get_absolute_time();

                    // Refresh WD resume info & marker only on flush boundary (wear-safe)
                    wd_store_state(true, current_filename);
                    (void)save_autolog_marker();
                }

                // ROLL AFTER N ROWS
                if (log_line_count >= LOG_ROLL_ROWS)
                {
                    close_current_log_file();
                    if (file_index < 99)
                        ++file_index;
                    else
                        file_index = 1;

                    if (!open_new_log_file())
                    {
                        // Fail-safe: stop logging to avoid data loss if we can't open next file
                        logging_active = false;
                        current_filename[0] = '\0';
                        wd_clear_state();
                        clear_autolog_marker();
                        printf("[LOG] Rolling failed; logging stopped.\n");
                        current_led_state = (mqtt_connected ? LED_CONNECTED : LED_MQTT_CONNECTING);
                        update_led_behavior();
                    }
                    else
                    {
                        base_time = get_absolute_time(); // optional: restart elapsed clock per file
                        last_flush = get_absolute_time();
                        last_adc_read = now;
                        (void)save_autolog_marker();
                    }
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
        // --- NETWORK SELF-HEAL ---
        if (mqtt_connected)
        {
            last_mqtt_ok_ms = now_ms;
        }
        else
        {
            if (now_ms - last_wifi_retry_ms > WIFI_RETRY_PERIOD_MS)
            {
                last_wifi_retry_ms = now_ms;
                (void)cyw43_arch_wifi_connect_timeout_ms(
                    cfg.wifi_ssid, cfg.wifi_password, CYW43_AUTH_WPA2_AES_PSK, 3000);
            }
            if (now_ms - last_mqtt_ok_ms > (NET_RETRY_WINDOW_MS + reboot_jitter_ms()))
            {
                printf("[NET] MQTT down > %u ms, rebooting to recover...\n", (unsigned)NET_RETRY_WINDOW_MS);
                (void)save_autolog_marker();
                wd_store_state(logging_active, current_filename);
                sleep_ms(50);
                watchdog_reboot(0, 0, 0);
                while (1)
                { /* wait */
                }
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

        if (now_ms - last_heater_ms >= HEATER_PERIOD_MS)
        {
            last_heater_ms = now_ms;
            heater_update_periodic(HEATER_PERIOD_MS);
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
