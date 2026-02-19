#pragma once

// Central place for *all* Pico-side pin assignments.
// Keep this file platform-agnostic: no Pico SDK headers required.

// =========================
// UART link to ESP
// =========================
// Choose uart0 or uart1 later; for now just use integers and map in code.
// In code you can do: uart_inst_t *u = (PICO_UART_ID==0)? uart0 : uart1;

#ifndef PICO_UART_ID
#define PICO_UART_ID 0 // 0 = uart0, 1 = uart1
#endif

#ifndef PICO_UART_BAUD
#define PICO_UART_BAUD 115200
#endif

#ifndef PICO_UART_TX_GPIO
#define PICO_UART_TX_GPIO 0 // set later (e.g. 4)
#endif

#ifndef PICO_UART_RX_GPIO
#define PICO_UART_RX_GPIO 1 // set later (e.g. 5)
#endif

// =========================
// DRV8163 motor driver pins
// =========================
// Required mapping decisions for real hardware bring-up:
// - DRV8163_CTRL_A_GPIO: PWM-capable pin driving IN1
// - DRV8163_CTRL_B_GPIO: PWM-capable pin driving IN2
// - DRV8163_SENSE_GPIO: ADC-capable pin connected to current sense
// - DRV8163_SENSE_ADC_CH: ADC channel index for DRV8163_SENSE_GPIO

#ifndef DRV8163_CTRL_A_GPIO
#define DRV8163_CTRL_A_GPIO 6 // PWM capable GPIO
#endif

#ifndef DRV8163_CTRL_B_GPIO
#define DRV8163_CTRL_B_GPIO 7 // PWM capable GPIO
#endif

#ifndef DRV8163_SENSE_GPIO
#define DRV8163_SENSE_GPIO 28 // ADC-capable GPIO (commonly 26/27/28 on RP2040-style)
#endif

// If you use RP2040-style ADC GPIO mapping, channel = gpio - 26.
// You can override this if needed.
#ifndef DRV8163_SENSE_ADC_CH
#define DRV8163_SENSE_ADC_CH ((DRV8163_SENSE_GPIO) - 26)
#endif

#ifndef DRV8163_DEFAULT_PWM_HZ
#define DRV8163_DEFAULT_PWM_HZ 20000
#endif

#ifndef DRV8163_DEFAULT_LOW_TH
#define DRV8163_DEFAULT_LOW_TH 30
#endif

#ifndef DRV8163_DEFAULT_HIGH_TH
#define DRV8163_DEFAULT_HIGH_TH 100
#endif

#ifndef DRV8163_DEFAULT_STARTUP_BLANKING_MS
#define DRV8163_DEFAULT_STARTUP_BLANKING_MS 250
#endif

#ifndef DRV8163_DEFAULT_CHECK_INTERVAL_MS
#define DRV8163_DEFAULT_CHECK_INTERVAL_MS 5
#endif

// =========================
// HX711 scale pins
// =========================
// Required mapping decisions for real hardware bring-up:
// - HX711_CLK_GPIO: Clock pin
// - HX711_DATA_GPIO: Data pin

#ifndef HX711_CLK_GPIO
#define HX711_CLK_GPIO 15
#endif

#ifndef HX711_DATA_GPIO
#define HX711_DATA_GPIO 14
#endif

#ifndef HX711_DEFAULT_RATE_US
#define HX711_DEFAULT_RATE_US 250000
#endif

// =========================
// DRV8434S stepper SPI
// =========================
// All DRV8434S devices in the daisy chain share the same MOSI/MISO/SCK and
// a single CS line.  Adjust pin numbers and DRV8434S_N_DEVICES to match your
// PCB layout.  Set DRV8434S_CS_GPIO to -1 (and SPI pins to -1) to disable
// the stepper subsystem at compile time.

#ifndef DRV8434S_SPI_ID
#define DRV8434S_SPI_ID 0 // 0 = spi0, 1 = spi1
#endif

#ifndef DRV8434S_SPI_BAUD
#define DRV8434S_SPI_BAUD 10000000 // 10 MHz (device supports up to 10 MHz)
#endif

#ifndef DRV8434S_SCK_GPIO
#define DRV8434S_SCK_GPIO 2
#endif

#ifndef DRV8434S_MOSI_GPIO
#define DRV8434S_MOSI_GPIO 3
#endif

#ifndef DRV8434S_MISO_GPIO
#define DRV8434S_MISO_GPIO 4
#endif

// Single shared chip-select for the entire daisy chain.
#ifndef DRV8434S_CS_GPIO
#define DRV8434S_CS_GPIO 5
#endif

// Number of DRV8434S devices wired in series.
// 1 = single device (still uses the daisy-chain framing, which is valid).
// Set to 0 to disable the stepper subsystem entirely.
#ifndef DRV8434S_N_DEVICES
#define DRV8434S_N_DEVICES 3
#endif

// =========================
// Optional compile-time guards
// =========================
// Turn these on once you're ready to require real pin selection.
#ifdef REQUIRE_REAL_PINS
#if (PICO_UART_TX_GPIO < 0) || (PICO_UART_RX_GPIO < 0)
#error "Set PICO_UART_TX_GPIO and PICO_UART_RX_GPIO in board_pins.h (or via -D defines)."
#endif

#if (DRV8163_CTRL_A_GPIO < 0) || (DRV8163_CTRL_B_GPIO < 0) || (DRV8163_SENSE_GPIO < 0)
#error "Set DRV8163_CTRL_A_GPIO / DRV8163_CTRL_B_GPIO / DRV8163_SENSE_GPIO in board_pins.h."
#endif

#if (DRV8163_SENSE_ADC_CH < 0) || (DRV8163_SENSE_ADC_CH > 3)
#error "DRV8163_SENSE_ADC_CH out of range (expected 0..3). Check ADC mapping."
#endif
#endif

/* ================================================================== */
/*  Stepper position constants (DRV8434S, steps from home)             */
/* ================================================================== */

/* Arm stepper (device 0) */
#ifndef ARM_STEPS_PRESS
#define ARM_STEPS_PRESS 500 /* steps to press attachment */
#endif
#ifndef ARM_STEPS_POS1
#define ARM_STEPS_POS1 1000 /* absolute position 1 */
#endif
#ifndef ARM_STEPS_POS2
#define ARM_STEPS_POS2 1500 /* absolute position 2 */
#endif
#ifndef ARM_PRESS_STALL_WINDOW_STEPS
#define ARM_PRESS_STALL_WINDOW_STEPS 50 /* steps to confirm press stall */
#endif
#ifndef ARM_MOTION_TIMEOUT_MS
#define ARM_MOTION_TIMEOUT_MS 5000
#endif

/* Rack stepper (device 1) */
#ifndef RACK_STEPS_EXTEND
#define RACK_STEPS_EXTEND 800
#endif
#ifndef RACK_STEPS_PRESS
#define RACK_STEPS_PRESS 1200
#endif
#ifndef RACK_MOTION_TIMEOUT_MS
#define RACK_MOTION_TIMEOUT_MS 5000
#endif

/* Turntable stepper (device 2) */
#ifndef TURNTABLE_STEPS_A
#define TURNTABLE_STEPS_A 0
#endif
#ifndef TURNTABLE_STEPS_B
#define TURNTABLE_STEPS_B 400
#endif
#ifndef TURNTABLE_STEPS_C
#define TURNTABLE_STEPS_C 800
#endif
#ifndef TURNTABLE_STEPS_D
#define TURNTABLE_STEPS_D 1200
#endif
#ifndef TURNTABLE_MOTION_TIMEOUT_MS
#define TURNTABLE_MOTION_TIMEOUT_MS 8000
#endif

/* ================================================================== */
/*  Flap (DRV8163 primary instance) thresholds                         */
/* ================================================================== */

#ifndef FLAP_OPEN_SPEED_PWM
#define FLAP_OPEN_SPEED_PWM 4095 /* full speed open */
#endif
#ifndef FLAP_OPEN_CURRENT_DROP_TH
#define FLAP_OPEN_CURRENT_DROP_TH 50 /* ADC counts -- open-circuit endpoint */
#endif
#ifndef FLAP_CLOSE_SPEED_PWM
#define FLAP_CLOSE_SPEED_PWM 1200 /* reduced speed on close */
#endif
#ifndef FLAP_CLOSE_TORQUE_TH
#define FLAP_CLOSE_TORQUE_TH 130 /* ADC counts -- stall/torque threshold */
#endif
#ifndef FLAP_MONITOR_INTERVAL_MS
#define FLAP_MONITOR_INTERVAL_MS 20
#endif
#ifndef FLAP_MOTION_TIMEOUT_MS
#define FLAP_MOTION_TIMEOUT_MS 8000
#endif

/* ================================================================== */
/*  Second flap linear actuator (DRV8163 instance 2)                   */
/* ================================================================== */
#ifndef FLAP2_CTRL_A_PIN
#define FLAP2_CTRL_A_PIN 8 /* GP8 */
#endif
#ifndef FLAP2_CTRL_B_PIN
#define FLAP2_CTRL_B_PIN 9 /* GP9 */
#endif
#ifndef FLAP2_ADC_SENSE_PIN
#define FLAP2_ADC_SENSE_PIN 27 /* GP28 — ADC channel 1 */
#endif
#ifndef FLAP2_ADC_CHANNEL
#define FLAP2_ADC_CHANNEL 1
#endif
/* Flap2 shares the same threshold constants as Flap1:
 * FLAP_OPEN_SPEED_PWM, FLAP_OPEN_CURRENT_DROP_TH,
 * FLAP_CLOSE_SPEED_PWM, FLAP_CLOSE_TORQUE_TH,
 * FLAP_MONITOR_INTERVAL_MS, FLAP_MOTION_TIMEOUT_MS
 */

/* ================================================================== */
/*  Hot wire (DRV8163 secondary instance)                              */
/* ================================================================== */

#ifndef HOTWIRE_PIN_IN1
#define HOTWIRE_PIN_IN1 10 /* GP11 -- PWM slice 5A */
#endif
#ifndef HOTWIRE_PIN_IN2
#define HOTWIRE_PIN_IN2 11 /* GP11 */
#endif
#ifndef HOTWIRE_ADC_SENSE_PIN
#define HOTWIRE_ADC_SENSE_PIN 26 /* GP26 -- ADC channel 0 */
#endif
#ifndef HOTWIRE_ADC_CHANNEL
#define HOTWIRE_ADC_CHANNEL 0
#endif
#ifndef HOTWIRE_CURRENT_DUTY
#define HOTWIRE_CURRENT_DUTY 4095 /* 40 % of 4095 -- constant current setpoint */
#endif
#ifndef HOTWIRE_MONITOR_INTERVAL_MS
#define HOTWIRE_MONITOR_INTERVAL_MS 50
#endif

/* ================================================================== */
/*  Vacuum pump                                                         */
/* ================================================================== */

#ifndef VACUUM_TRIGGER_PIN
#define VACUUM_TRIGGER_PIN 12 /* GP12 -- digital output */
#endif
#ifndef VACUUM_RPM_SENSE_PIN
#define VACUUM_RPM_SENSE_PIN 13 /* GP13 -- rising-edge interrupt input */
#endif
#ifndef VACUUM_PULSES_PER_REV
#define VACUUM_PULSES_PER_REV 1 /* adjust to match pump sensor */
#endif
#ifndef VACUUM_RPM_SAMPLE_MS
#define VACUUM_RPM_SAMPLE_MS 100 /* RPM sampling window */
#endif
#ifndef VACUUM_RPM_BLOCKED_THRESHOLD
#define VACUUM_RPM_BLOCKED_THRESHOLD 400 /* RPM below this = blocked */
#endif
#ifndef VACUUM_STATUS_SEND_INTERVAL_MS
#define VACUUM_STATUS_SEND_INTERVAL_MS 5000 /* unsolicited status period */
#endif
