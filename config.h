#pragma once
#ifndef CONFIG_H
#define CONFIG_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

/* Pico SDK also provides fixed-width types; this helps some IntelliSense setups */
#include "pico/types.h"

#include "sd_utils.h" // for sd_read_text()

#ifndef CONFIG_PATH
#define CONFIG_PATH "CFG.TXT"
#endif

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        char device_name[32];
        char wifi_ssid[64];
        char wifi_password[64];
        char broker_ip[64];

        uint32_t sample_ms;  // ADC sample period in ms (runtime override)
        uint8_t log_flush_s; // log flush interval seconds
        uint8_t cfg_version; // schema version
    } device_config_t;

    /* Initialize config struct with compile-time defaults (DEVICE_NAME, WIFI_SSID, etc.) */
    void config_init_defaults(device_config_t *cfg);

    /* Load config from SD file (path). Returns true if file found & parsed, false otherwise. */
    bool config_load(device_config_t *cfg, const char *path);

    /* Request/reload API (set by MQTT command handler) */
    void config_request_reload(void);
    bool config_reload_requested(void);
    void config_clear_reload_request(void);

    /* Convenience accessor (returns pointer to last loaded / current config) */
    const device_config_t *config_get(void);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // CONFIG_H
