#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "seacoast_proto.h"

// Choose UART + pins that are NOT your console UART.
// On ESP32-C6, UART0 is usually console on GPIO16/17 (per your log).
#define SC_UART_NUM      UART_NUM_1

// Pick two free GPIOs on your board for TX/RX to Pico.
#define SC_UART_TX_GPIO  4
#define SC_UART_RX_GPIO  5

#define SC_UART_BAUD     460800

esp_err_t uart_link_init(void);
esp_err_t uart_link_send(uint8_t type, const void *payload, uint8_t len);

// Optional: a simple poll-style receiver (non-blocking)
// Call from a task if you want to process incoming STATUS/ACK frames.
void uart_link_rx_pump(void (*on_frame)(const sc_frame_view_t *f));
