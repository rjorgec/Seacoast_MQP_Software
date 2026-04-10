#include "pico_link.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "proto.h"
#include "cobs.h"

#define LINK_ENCODED_FRAME_MAX ((size_t)sizeof(proto_hdr_t) + PROTO_MAX_PAYLOAD + 8u)
#define LINK_DECODED_FRAME_MAX ((size_t)sizeof(proto_hdr_t) + PROTO_MAX_PAYLOAD + 2u)

static const char *TAG = "pico_link";

typedef struct {
    bool active;
    uint16_t seq;
    esp_err_t result;
    uint8_t nack_code;
} pending_rpc_t;

static pico_link_cfg_t s_cfg;
static bool s_inited;
static uint16_t s_next_seq = 1u;

static SemaphoreHandle_t s_lock;
static SemaphoreHandle_t s_rpc_done;
static pending_rpc_t s_pending;

static esp_err_t link_send_frame(uint8_t msg_type, uint16_t seq, const void *payload, uint16_t len) {
    uint8_t raw[LINK_DECODED_FRAME_MAX];
    uint8_t enc[LINK_ENCODED_FRAME_MAX];

    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }

    if (len > PROTO_MAX_PAYLOAD) {
        return ESP_ERR_INVALID_ARG;
    }

    proto_hdr_t hdr = {
        .version = PROTO_VERSION,
        .type = msg_type,
        .seq = seq,
        .len = len,
    };

    memcpy(raw, &hdr, sizeof(hdr));
    if (payload != NULL && len > 0u) {
        memcpy(raw + sizeof(hdr), payload, len);
    }

    const size_t raw_len = sizeof(hdr) + len + 2u;
    const uint16_t crc = proto_crc16_ccitt(raw, (uint32_t)(sizeof(hdr) + len));
    raw[sizeof(hdr) + len] = (uint8_t)(crc & 0xFFu);
    raw[sizeof(hdr) + len + 1u] = (uint8_t)((crc >> 8) & 0xFFu);

    const size_t enc_len = cobs_encode(raw, raw_len, enc);
    if (enc_len == 0u || enc_len > sizeof(enc)) {
        return ESP_FAIL;
    }

    const int written = uart_write_bytes(s_cfg.uart_num, (const char *)enc, enc_len);
    if (written != (int)enc_len) {
        return ESP_FAIL;
    }

    const uint8_t delim = PROTO_DELIM;
    if (uart_write_bytes(s_cfg.uart_num, (const char *)&delim, 1) != 1) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

static void handle_incoming_frame(const uint8_t *encoded, size_t encoded_len) {
    uint8_t decoded[LINK_DECODED_FRAME_MAX];

    const size_t decoded_len = cobs_decode(encoded, encoded_len, decoded);
    if (decoded_len < sizeof(proto_hdr_t) + 2u || decoded_len > sizeof(decoded)) {
        return;
    }

    proto_hdr_t hdr;
    memcpy(&hdr, decoded, sizeof(hdr));

    const uint16_t got_crc = (uint16_t)decoded[decoded_len - 2u] |
                             ((uint16_t)decoded[decoded_len - 1u] << 8);
    const uint16_t calc_crc = proto_crc16_ccitt(decoded, (uint32_t)(decoded_len - 2u));
    if (got_crc != calc_crc) {
        return;
    }

    if (hdr.version != PROTO_VERSION ||
        hdr.len > PROTO_MAX_PAYLOAD ||
        decoded_len != (size_t)sizeof(proto_hdr_t) + hdr.len + 2u) {
        return;
    }

    const uint8_t *payload = decoded + sizeof(proto_hdr_t);

    if (xSemaphoreTake(s_lock, portMAX_DELAY) == pdTRUE) {
        if (s_pending.active && hdr.seq == s_pending.seq && (hdr.type == MSG_ACK || hdr.type == MSG_NACK)) {
            if (hdr.type == MSG_ACK) {
                s_pending.result = ESP_OK;
                s_pending.nack_code = 0u;
            } else {
                s_pending.result = ESP_FAIL;
                s_pending.nack_code = (hdr.len == sizeof(pl_nack_t)) ? payload[0] : (uint8_t)NACK_UNKNOWN;
            }

            s_pending.active = false;
            xSemaphoreGive(s_rpc_done);
            xSemaphoreGive(s_lock);
            return;
        }
        xSemaphoreGive(s_lock);
    }

    if (s_cfg.on_rx != NULL) {
        s_cfg.on_rx(hdr.type, hdr.seq, payload, hdr.len);
    }
}

static void pico_link_rx_task(void *arg) {
    (void)arg;

    uint8_t frame[LINK_ENCODED_FRAME_MAX];
    size_t frame_len = 0u;
    bool overflow = false;

    while (1) {
        uint8_t b = 0u;
        const int n = uart_read_bytes(s_cfg.uart_num, &b, 1u, pdMS_TO_TICKS(50));
        if (n <= 0) {
            continue;
        }

        if (b == PROTO_DELIM) {
            if (!overflow && frame_len > 0u) {
                handle_incoming_frame(frame, frame_len);
            }
            frame_len = 0u;
            overflow = false;
            continue;
        }

        if (overflow) {
            continue;
        }

        if (frame_len >= sizeof(frame)) {
            overflow = true;
            continue;
        }

        frame[frame_len++] = b;
    }
}

esp_err_t pico_link_init(const pico_link_cfg_t *cfg) {
    if (cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_inited) {
        return ESP_OK;
    }

    s_cfg = *cfg;

    uart_config_t uc = {
        .baud_rate = s_cfg.baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(s_cfg.uart_num, &uc));
    ESP_ERROR_CHECK(uart_set_pin(s_cfg.uart_num,
                                 s_cfg.tx_gpio,
                                 s_cfg.rx_gpio,
                                 UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(s_cfg.uart_num, 4096, 4096, 0, NULL, 0));

    s_lock = xSemaphoreCreateMutex();
    s_rpc_done = xSemaphoreCreateBinary();
    if (s_lock == NULL || s_rpc_done == NULL) {
        return ESP_ERR_NO_MEM;
    }

    memset(&s_pending, 0, sizeof(s_pending));

    BaseType_t ok = xTaskCreate(pico_link_rx_task, "pico_link_rx", 4096, NULL, 10, NULL);
    if (ok != pdTRUE) {
        return ESP_FAIL;
    }

    s_inited = true;
    ESP_LOGI(TAG, "UART link up: uart=%d tx=%d rx=%d baud=%d",
             (int)s_cfg.uart_num,
             s_cfg.tx_gpio,
             s_cfg.rx_gpio,
             s_cfg.baud);
    return ESP_OK;
}

esp_err_t pico_link_send(uint8_t msg_type, const void *payload, uint16_t len, uint16_t *out_seq) {
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }

    if (len > PROTO_MAX_PAYLOAD) {
        return ESP_ERR_INVALID_ARG;
    }

    if (len > 0u && payload == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t seq = 0u;
    if (xSemaphoreTake(s_lock, portMAX_DELAY) == pdTRUE) {
        seq = s_next_seq++;
        xSemaphoreGive(s_lock);
    }

    esp_err_t err = link_send_frame(msg_type, seq, payload, len);
    if (err != ESP_OK) {
        return err;
    }

    if (out_seq != NULL) {
        *out_seq = seq;
    }

    return ESP_OK;
}

esp_err_t pico_link_send_rpc(uint8_t msg_type,
                             const void *payload,
                             uint16_t len,
                             uint32_t timeout_ms,
                             uint8_t *out_nack_code) {
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }

    if (len > PROTO_MAX_PAYLOAD) {
        return ESP_ERR_INVALID_ARG;
    }

    if (len > 0u && payload == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (timeout_ms == 0u) {
        return ESP_ERR_INVALID_ARG;
    }

    if (out_nack_code != NULL) {
        *out_nack_code = 0u;
    }

    if (xSemaphoreTake(s_lock, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }

    if (s_pending.active) {
        xSemaphoreGive(s_lock);
        return ESP_ERR_INVALID_STATE;
    }

    while (xSemaphoreTake(s_rpc_done, 0) == pdTRUE) {
    }

    const uint16_t seq = s_next_seq++;
    s_pending.active = true;
    s_pending.seq = seq;
    s_pending.result = ESP_ERR_TIMEOUT;
    s_pending.nack_code = 0u;

    xSemaphoreGive(s_lock);

    esp_err_t err = link_send_frame(msg_type, seq, payload, len);
    if (err != ESP_OK) {
        if (xSemaphoreTake(s_lock, portMAX_DELAY) == pdTRUE) {
            s_pending.active = false;
            xSemaphoreGive(s_lock);
        }
        return err;
    }

    if (xSemaphoreTake(s_rpc_done, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        if (xSemaphoreTake(s_lock, portMAX_DELAY) == pdTRUE) {
            s_pending.active = false;
            xSemaphoreGive(s_lock);
        }
        return ESP_ERR_TIMEOUT;
    }

    if (xSemaphoreTake(s_lock, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }

    err = s_pending.result;
    if (out_nack_code != NULL) {
        *out_nack_code = s_pending.nack_code;
    }
    xSemaphoreGive(s_lock);

    return err;
}
