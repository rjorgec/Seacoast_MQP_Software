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
#define DRV8163_CTRL_A_GPIO 20 // PWM capable GPIO
#endif

#ifndef DRV8163_CTRL_B_GPIO
#define DRV8163_CTRL_B_GPIO 21 // PWM capable GPIO
#endif

#ifndef DRV8163_SENSE_GPIO
#define DRV8163_SENSE_GPIO 27 // ADC-capable GPIO (commonly 26/27/28 on RP2040-style)
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
