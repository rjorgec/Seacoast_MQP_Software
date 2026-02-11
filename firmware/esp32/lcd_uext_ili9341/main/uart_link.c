#include "uart_link.h"
#include "driver/uart.h"
#include "esp_log.h"

static const char *TAG = "uart_link";

static sc_parser_t s_parser;

esp_err_t uart_link_init(void)
{
    sc_parser_init(&s_parser);

    uart_config_t cfg = {
        .baud_rate = SC_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(SC_UART_NUM, 2048, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(SC_UART_NUM, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(SC_UART_NUM, SC_UART_TX_GPIO, SC_UART_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "UART link up: uart=%d tx=%d rx=%d baud=%d",
             (int)SC_UART_NUM, SC_UART_TX_GPIO, SC_UART_RX_GPIO, SC_UART_BAUD);
    return ESP_OK;
}

esp_err_t uart_link_send(uint8_t type, const void *payload, uint8_t len)
{
    uint8_t buf[2 + 2 + SC_MAX_PAYLOAD + 2];
    size_t n = sc_build_frame(type, payload, len, buf, sizeof(buf));
    if (n == 0) return ESP_ERR_INVALID_SIZE;

    int w = uart_write_bytes(SC_UART_NUM, (const char *)buf, (int)n);
    return (w == (int)n) ? ESP_OK : ESP_FAIL;
}

void uart_link_rx_pump(void (*on_frame)(const sc_frame_view_t *f))
{
    uint8_t tmp[128];
    int r = uart_read_bytes(SC_UART_NUM, tmp, sizeof(tmp), 0); // non-blocking
    if (r <= 0) return;

    for (int i = 0; i < r; i++) {
        sc_frame_view_t f;
        if (sc_parser_feed(&s_parser, tmp[i], &f)) {
            if (on_frame) on_frame(&f);
        }
    }
}
