#pragma once
#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

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

        uint8_t heater_enabled;      // 0 = disabled, 1 = enabled
        uint8_t heater_pwm_pin;      // PWM output pin (default GP2)
        uint8_t heater_tc_cs_pin;    // TC chip-select pin (default GP5)
        uint8_t heater_control_mode; // 0 = ON/OFF, 1 = PI
        float heater_target_c;       // target temperature in °C
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
