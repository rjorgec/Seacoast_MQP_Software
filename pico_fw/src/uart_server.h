#pragma once
#include <stdbool.h>
#include <stdint.h>
#define TUNING

/* Phase 1 safe-state: configure every output GPIO the firmware will use, with
 * the level each signal needs to take before 12 V is applied to the motor
 * driver ICs. Must be called at the very top of main() before stdio_init_all()
 * and any boot delay. The critical pin is HOTWIRE_PIN_IN2 (held HIGH so the
 * ground-side-wired pump 2 stays off when the ESP later closes the relay). */
void uart_server_phase1_pinmodes(void);

/* Phase 1 communication bringup: UART + ring buffer + PING handler. Runs
 * before the 12 V relay closes, so it deliberately does not touch the motor
 * drivers, HX711, or the DRV8434S SPI chain. After the ESP's PING has been
 * ACKed, uart_server_poll() schedules the Phase 3 late init automatically. */
void uart_server_init_early(void);

void uart_server_poll(void); // call frequently in main loop
