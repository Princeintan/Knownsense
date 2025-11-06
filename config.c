#include "config.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stddef.h> // <-- for size_t
#include <stdint.h> // <-- belt-and-suspenders for uint32_t/uint8_t
#include <stdbool.h>

/* JSON helpers: tiny parsers for flat JSON with string values and unsigned ints.
   Not a full JSON parser but fine for simple config files. */

static const char *skip_ws(const char *p)
{
    while (p && (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n'))
        p++;
    return p;
}

static bool json_get_string(const char *json, const char *key, char *out, size_t outlen)
{
    if (!json || !key || !out || outlen < 2)
        return false;
    char pat[96];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char *k = strstr(json, pat);
    if (!k)
        return false;

    const char *p = k + strlen(pat);
    p = skip_ws(p);
    if (*p != ':')
        return false;
    p = skip_ws(p + 1);
    if (*p != '\"')
        return false;
    p++; // inside string

    size_t n = 0;
    while (*p && *p != '\"' && n + 1 < outlen)
    {
        if (*p == '\\' && p[1] != '\0')
            p++; // rudimentary unescape
        out[n++] = *p++;
    }
    out[n] = '\0';
    return (*p == '\"');
}

static bool json_get_uint(const char *json, const char *key, unsigned *out)
{
    if (!json || !key || !out)
        return false;
    char pat[96];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char *k = strstr(json, pat);
    if (!k)
        return false;

    const char *p = k + strlen(pat);
    p = skip_ws(p);
    if (*p != ':')
        return false;
    p = skip_ws(p + 1);

    // parse integer (decimal)
    unsigned v = 0;
    bool seen = false;
    while (*p >= '0' && *p <= '9')
    {
        seen = true;
        v = v * 10 + (unsigned)(*p - '0');
        p++;
    }
    if (!seen)
        return false;
    *out = v;
    return true;
}

/* Module state */
static device_config_t current_cfg;
static volatile bool reload_requested = false;

/* Provide compile-time fallbacks if not defined */
#ifndef DEVICE_NAME
#define DEVICE_NAME "K02"
#endif

#ifndef WIFI_SSID
#define WIFI_SSID ""
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD ""
#endif

#ifndef MQTT_SERVER
#define MQTT_SERVER ""
#endif

#ifndef ADC_READ_INTERVAL_MS
#define ADC_READ_INTERVAL_MS 1000
#endif

void config_init_defaults(device_config_t *cfg)
{
    if (!cfg)
        return;
    memset(cfg, 0, sizeof(*cfg));
    snprintf(cfg->device_name, sizeof(cfg->device_name), "%s", DEVICE_NAME);
    snprintf(cfg->wifi_ssid, sizeof(cfg->wifi_ssid), "%s", WIFI_SSID);
    snprintf(cfg->wifi_password, sizeof(cfg->wifi_password), "%s", WIFI_PASSWORD);
    snprintf(cfg->broker_ip, sizeof(cfg->broker_ip), "%s", MQTT_SERVER);

    cfg->sample_ms = (uint32_t)ADC_READ_INTERVAL_MS;
    cfg->log_flush_s = 1; // default 1 second flush
    cfg->cfg_version = 1;

    /* copy to module current config */
    memcpy(&current_cfg, cfg, sizeof(*cfg));
}

bool config_load(device_config_t *cfg, const char *path)
{
    if (!cfg || !path)
        return false;

    /* start from existing cfg (so missing keys keep previous value) */
    device_config_t tmp;
    memcpy(&tmp, cfg, sizeof(tmp));

    /* read file via sd_utils */
    static char json_buf[2048];
    if (!sd_read_text(path, json_buf, sizeof(json_buf)))
    {
        // file not found -> keep provided defaults
        printf("[CFG] %s not found; using defaults/current values.\n", path);
        // persist current values to module state
        memcpy(&current_cfg, &tmp, sizeof(tmp));
        return false;
    }

    /* parse strings (best-effort) */
    (void)json_get_string(json_buf, "device_name", tmp.device_name, sizeof(tmp.device_name));
    (void)json_get_string(json_buf, "wifi_ssid", tmp.wifi_ssid, sizeof(tmp.wifi_ssid));
    (void)json_get_string(json_buf, "wifi_password", tmp.wifi_password, sizeof(tmp.wifi_password));
    (void)json_get_string(json_buf, "broker_ip", tmp.broker_ip, sizeof(tmp.broker_ip));

    /* parse unsigned numeric fields */
    unsigned u = 0;
    if (json_get_uint(json_buf, "sample_ms", &u))
        tmp.sample_ms = (uint32_t)u;
    if (json_get_uint(json_buf, "log_flush_s", &u))
        tmp.log_flush_s = (uint8_t)u;
    if (json_get_uint(json_buf, "cfg_version", &u))
        tmp.cfg_version = (uint8_t)u;

    /* commit parsed values */
    memcpy(cfg, &tmp, sizeof(tmp));
    memcpy(&current_cfg, &tmp, sizeof(tmp));

    printf("[CFG] Loaded %s v%u (dev=%s, broker=%s, sample_ms=%ums, flush=%us)\n",
           path, (unsigned)tmp.cfg_version, tmp.device_name, tmp.broker_ip, (unsigned)tmp.sample_ms, (unsigned)tmp.log_flush_s);

    return true;
}

/* reload API */
void config_request_reload(void)
{
    reload_requested = true;
}
bool config_reload_requested(void)
{
    return reload_requested;
}
void config_clear_reload_request(void)
{
    reload_requested = false;
}

const device_config_t *config_get(void)
{
    return &current_cfg;
}
