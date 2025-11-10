// heater.h
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "pico/mutex.h"

// Register a pointer to a global SPI mutex so heater and other modules can coordinate.
// Pass the address of your global mutex_t (e.g. &spi_mutex).
void heater_register_spi_mutex(mutex_t *m);

// Initialize heater module if hardware present. Call once after gpio/spi are available.
// pwm_pin = GPIO for PWM output (e.g. 2), tc_cs_pin = CS pin for thermocouple (e.g. 5).
// control_mode: 0 = on/off, 1 = PI. target_c = initial target temperature in °C.
void heater_init_if_enabled(uint8_t pwm_pin, uint8_t tc_cs_pin, uint8_t control_mode, float target_c);

// Set/get target temperature (°C)
void heater_set_target_c(float t);
float heater_get_target_c(void);

// Periodic control update: dt_ms = milliseconds since last call (call every ~500ms).
void heater_update_periodic(uint32_t dt_ms);

// Force heater OFF (set PWM = 0)
void heater_force_off(void);

// Returns true if thermocouple successfully read previously
bool heater_present_tc(void);
