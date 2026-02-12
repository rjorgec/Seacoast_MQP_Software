#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/uart.h"   // <- provides uart_port_t and UART_NUM_1

typedef void (*pico_link_rx_cb_t)(uint8_t msg_type,
                                 uint16_t seq,
                                 const uint8_t *payload,
                                 uint16_t len);

typedef struct {
    uart_port_t uart_num;   // <- so UART_NUM_1 works
    int tx_gpio;
    int rx_gpio;
    int baud;
    pico_link_rx_cb_t on_rx;
} pico_link_cfg_t;

esp_err_t pico_link_init(const pico_link_cfg_t *cfg);
esp_err_t pico_link_send(uint8_t msg_type, const void *payload, uint16_t len, uint16_t *out_seq);
