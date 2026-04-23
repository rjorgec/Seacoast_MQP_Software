#pragma once

#include <stdbool.h>

#include "esp_err.h"

/*
 * 12 V motor-driver power rail control (GPIO10 → relay coil).
 *
 * GPIO10 drives the coil of a normally-open relay whose contact gates 12 V to
 * every motor driver IC (DRV8424s and DRV8263). The ESP boots with the relay
 * open (motor drivers unpowered), performs the PING handshake with the Pico,
 * then closes the relay. The relay is dropped again on SYS_ESTOP.
 */

/* Configure GPIO10 as a push-pull output, driven LOW. Must be called before
 * pico_link_init() so the relay is guaranteed open while the ESP is still
 * bringing up UART and UI. */
esp_err_t pico_power_init(void);

/* Close the relay (drive GPIO10 HIGH). Called after the Pico has ACKed the
 * boot-time PING and is ready for 12 V to appear on the motor driver inputs. */
esp_err_t pico_power_enable(void);

/* Open the relay (drive GPIO10 LOW). Called on SYS_ESTOP to drop motor-driver
 * power immediately. */
esp_err_t pico_power_disable(void);

/* True if the relay is currently commanded closed. */
bool pico_power_is_enabled(void);
