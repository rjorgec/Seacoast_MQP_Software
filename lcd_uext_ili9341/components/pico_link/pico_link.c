#include "pico_link.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "pico_link";

static pico_link_cfg_t s_cfg;
static bool s_inited = false;

// Simple RX framing buffer (0x00 delimiter)
static void pico_link_rx_task(void *arg) {
    (void)arg;

    uint8_t frame[256];
    size_t frame_len = 0;

    while (1) {
        uint8_t b = 0;
        int n = uart_read_bytes(s_cfg.uart_num, &b, 1, pdMS_TO_TICKS(50));
        if (n <= 0) continue;

        if (b == 0x00) {
            // End of frame
            if (frame_len > 0) {
                // TODO: decode (COBS) + CRC + dispatch
                // For now, just log that we received something.
                ESP_LOGD(TAG, "RX frame len=%u", (unsigned)frame_len);

                // If you already have a decoder, call s_cfg.on_rx(...) here.
                // Example placeholder:
                // if (s_cfg.on_rx) s_cfg.on_rx(type, seq, payload, len);
            }
            frame_len = 0;
            continue;
        }

        if (frame_len < sizeof(frame)) {
            frame[frame_len++] = b;
        } else {
            // overflow -> drop
            ESP_LOGW(TAG, "RX frame overflow, dropping");
            frame_len = 0;
        }
    }
}

esp_err_t pico_link_init(const pico_link_cfg_t *cfg) {
    if (!cfg) return ESP_ERR_INVALID_ARG;
    if (s_inited) return ESP_OK;

    s_cfg = *cfg;

    uart_config_t uc = {
        .baud_rate = s_cfg.baud,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(s_cfg.uart_num, &uc));
    ESP_ERROR_CHECK(uart_set_pin(s_cfg.uart_num, s_cfg.tx_gpio, s_cfg.rx_gpio,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // RX/TX buffers
    ESP_ERROR_CHECK(uart_driver_install(s_cfg.uart_num, 4096, 4096, 0, NULL, 0));

    BaseType_t ok = xTaskCreate(pico_link_rx_task, "pico_link_rx", 4096, NULL, 10, NULL);
    if (ok != pdTRUE) return ESP_FAIL;

    s_inited = true;
    ESP_LOGI(TAG, "UART link up: uart=%d tx=%d rx=%d baud=%d",
             (int)s_cfg.uart_num, s_cfg.tx_gpio, s_cfg.rx_gpio, s_cfg.baud);
    return ESP_OK;
}

esp_err_t pico_link_send(uint8_t msg_type, const void *payload, uint16_t len, uint16_t *out_seq) {
    // TODO: Replace with proto frame (hdr + crc + COBS + 0x00)
    // For now, just send raw bytes with a delimiter so you can prove the wire works.

    (void)out_seq;

    uint8_t header[2] = { msg_type, (uint8_t)len };
    uart_write_bytes(s_cfg.uart_num, (const char*)header, sizeof(header));
    if (payload && len) {
        uart_write_bytes(s_cfg.uart_num, (const char*)payload, len);
    }
    uint8_t z = 0x00;
    uart_write_bytes(s_cfg.uart_num, (const char*)&z, 1);

    return ESP_OK;
}
