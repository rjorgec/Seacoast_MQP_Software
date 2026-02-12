#include "uart_server.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/uart.h"

#include "board_pins.h"
#include "drivers/drv8163/drv8163.h"
#include "shared/proto.h"
#include "shared/cobs.h"

#define UART_RX_RING_SIZE 512u
#define UART_ENCODED_FRAME_MAX 256u
#define UART_DECODED_FRAME_MAX ((size_t)sizeof(proto_hdr_t) + PROTO_MAX_PAYLOAD + 2u)

typedef struct
{
    uint8_t buf[UART_RX_RING_SIZE];
    uint16_t head;
    uint16_t tail;
    uint16_t count;
} rx_ring_t;

static rx_ring_t s_rx_ring;
static uint8_t s_frame_buf[UART_ENCODED_FRAME_MAX];
static size_t s_frame_len;
static bool s_frame_overflow;

static drv8163_t s_drv8163;
static bool s_drv8163_ready;

static uart_inst_t *s_uart;
static bool s_uart_ready;

static uart_inst_t *uart_from_id(void)
{
    return (PICO_UART_ID == 0) ? uart0 : uart1;
}

static bool rx_ring_push(uint8_t b)
{
    if (s_rx_ring.count >= UART_RX_RING_SIZE)
    {
        return false;
    }

    s_rx_ring.buf[s_rx_ring.head] = b;
    s_rx_ring.head = (uint16_t)((s_rx_ring.head + 1u) % UART_RX_RING_SIZE);
    ++s_rx_ring.count;
    return true;
}

static bool rx_ring_pop(uint8_t *out)
{
    if (s_rx_ring.count == 0u || out == NULL)
    {
        return false;
    }

    *out = s_rx_ring.buf[s_rx_ring.tail];
    s_rx_ring.tail = (uint16_t)((s_rx_ring.tail + 1u) % UART_RX_RING_SIZE);
    --s_rx_ring.count;
    return true;
}

static bool uart_send_frame(uint8_t type, uint16_t seq, const void *payload, uint16_t len)
{
    uint8_t raw[UART_DECODED_FRAME_MAX];
    uint8_t enc[UART_ENCODED_FRAME_MAX];

    if (len > PROTO_MAX_PAYLOAD)
    {
        return false;
    }

    proto_hdr_t hdr = {
        .version = PROTO_VERSION,
        .type = type,
        .seq = seq,
        .len = len,
    };

    memcpy(raw, &hdr, sizeof(hdr));
    if (payload != NULL && len > 0u)
    {
        memcpy(raw + sizeof(hdr), payload, len);
    }

    const size_t raw_len = sizeof(hdr) + len + 2u;
    uint16_t crc = proto_crc16_ccitt(raw, (uint32_t)(sizeof(hdr) + len));
    raw[sizeof(hdr) + len] = (uint8_t)(crc & 0xFFu);
    raw[sizeof(hdr) + len + 1u] = (uint8_t)((crc >> 8) & 0xFFu);

    const size_t enc_len = cobs_encode(raw, raw_len, enc);
    if (enc_len == 0u || enc_len > sizeof(enc))
    {
        return false;
    }

    uart_write_blocking(s_uart, enc, enc_len);
    const uint8_t delim = PROTO_DELIM;
    uart_write_blocking(s_uart, &delim, 1u);
    return true;
}

static void send_ack(uint16_t seq)
{
    (void)uart_send_frame(MSG_ACK, seq, NULL, 0u);
}

static void send_nack(uint16_t seq, nack_code_t code)
{
    pl_nack_t nack = {.code = (uint8_t)code};
    (void)uart_send_frame(MSG_NACK, seq, &nack, (uint16_t)sizeof(nack));
}

static void handle_drv8163_start(uint16_t seq, const uint8_t *payload, uint16_t len)
{
    if (len != (uint16_t)sizeof(pl_drv8163_start_mon_t))
    {
        send_nack(seq, NACK_BAD_LEN);
        return;
    }

    if (!s_drv8163_ready)
    {
        send_nack(seq, NACK_UNKNOWN);
        return;
    }

    const pl_drv8163_start_mon_t *p = (const pl_drv8163_start_mon_t *)payload;

    s_drv8163.config.current_low_threshold = p->low_th;
    s_drv8163.config.current_high_threshold = p->high_th;
    s_drv8163.config.current_check_interval_ms = p->interval_ms;

    drv8163_motor_state_t st = (p->dir == 0) ? DRV8163_MOTOR_FORWARD : DRV8163_MOTOR_REVERSE; //new
    (void)drv8163_set_motor_control(&s_drv8163, st, p->speed);


    if (s_drv8163.monitoring_enabled)
    {
        drv8163_stop_current_monitoring(&s_drv8163);
    }

    if (!drv8163_start_current_monitoring(&s_drv8163))
    {
        printf("ADC %u\n", s_drv8163.last_current_adc.average);
        send_nack(seq, NACK_UNKNOWN);
        return;
    }

    send_ack(seq);
}

static void handle_drv8163_stop(uint16_t seq)
{
    if (!s_drv8163_ready)
    {
        send_nack(seq, NACK_UNKNOWN);
        return;
    }

    drv8163_stop_current_monitoring(&s_drv8163);
    (void)drv8163_set_motor_control(&s_drv8163, DRV8163_MOTOR_STOP, 0u);
    send_ack(seq);
}

static void dispatch_decoded(const uint8_t *decoded, size_t decoded_len)
{
    if (decoded_len < sizeof(proto_hdr_t) + 2u)
    {
        return;
    }
    printf("message recieved\n");
    proto_hdr_t hdr;
    memcpy(&hdr, decoded, sizeof(hdr));

    const uint16_t got_crc = (uint16_t)decoded[decoded_len - 2u] |
                             ((uint16_t)decoded[decoded_len - 1u] << 8);
    const uint16_t calc_crc = proto_crc16_ccitt(decoded, (uint32_t)(decoded_len - 2u));

    if (got_crc != calc_crc)
    {
        send_nack(hdr.seq, NACK_BAD_CRC);
        return;
    }

    if (hdr.version != PROTO_VERSION)
    {
        send_nack(hdr.seq, NACK_BAD_VERSION);
        return;
    }

    if (hdr.len > PROTO_MAX_PAYLOAD)
    {
        send_nack(hdr.seq, NACK_BAD_LEN);
        return;
    }

    if (decoded_len != (size_t)sizeof(proto_hdr_t) + hdr.len + 2u)
    {
        send_nack(hdr.seq, NACK_BAD_LEN);
        return;
    }

    const uint8_t *payload = decoded + sizeof(proto_hdr_t);

    switch ((msg_type_t)hdr.type)
    {
    case MSG_PING:
        printf("Ping\n");
        send_ack(hdr.seq);
        break;

    case MSG_MOTOR_DRV8163_START_MON:
        printf("Start\n");
        handle_drv8163_start(hdr.seq, payload, hdr.len);
        break;

    case MSG_MOTOR_DRV8163_STOP_MON:
        printf("Stop\n");
        handle_drv8163_stop(hdr.seq);
        break;

    default:
        send_nack(hdr.seq, NACK_UNKNOWN);
        break;
    }
}

static void process_frame(void)
{
    uint8_t decoded[UART_DECODED_FRAME_MAX];

    if (s_frame_len == 0u || s_frame_overflow)
    {
        s_frame_len = 0u;
        s_frame_overflow = false;
        return;
    }

    const size_t decoded_len = cobs_decode(s_frame_buf, s_frame_len, decoded);
    if (decoded_len == 0u || decoded_len > sizeof(decoded))
    {
        s_frame_len = 0u;
        return;
    }

    dispatch_decoded(decoded, decoded_len);
    s_frame_len = 0u;
}

static void init_drv8163(void)
{
    s_drv8163_ready = false;

    if (DRV8163_CTRL_A_GPIO < 0 || DRV8163_CTRL_B_GPIO < 0 || DRV8163_SENSE_GPIO < 0 ||
        DRV8163_SENSE_ADC_CH < 0 || DRV8163_SENSE_ADC_CH > 3)
    {
        printf("uart_server: DRV8163 pins/ch not set in board_pins.h; motor RPC disabled\n");
        return;
    }

    drv8163_config_t cfg = {
        .user_ctx = NULL,
        .ctrl_a_pin = (uint8_t)DRV8163_CTRL_A_GPIO,
        .ctrl_b_pin = (uint8_t)DRV8163_CTRL_B_GPIO,
        .sense_pin = (uint8_t)DRV8163_SENSE_GPIO,
        .sense_adc_channel = (uint8_t)DRV8163_SENSE_ADC_CH,
        .current_high_threshold = DRV8163_DEFAULT_HIGH_TH,
        .current_low_threshold = DRV8163_DEFAULT_LOW_TH,
        .pwm_frequency_hz = DRV8163_DEFAULT_PWM_HZ,
        .current_check_interval_ms = DRV8163_DEFAULT_CHECK_INTERVAL_MS,
        .startup_blanking_ms = DRV8163_DEFAULT_STARTUP_BLANKING_MS,
        .current_cb = NULL,
        .delay_ms = NULL,
    };

    if (!drv8163_init(&s_drv8163, &cfg))
    {
        printf("uart_server: DRV8163 init failed\n");
        return;
    }

    s_drv8163_ready = true;
}

void uart_server_init(void)
{
    memset(&s_rx_ring, 0, sizeof(s_rx_ring));
    s_frame_len = 0u;
    s_frame_overflow = false;

    s_uart = uart_from_id();
    s_uart_ready = false;

    if (PICO_UART_TX_GPIO < 0 || PICO_UART_RX_GPIO < 0)
    {
        printf("uart_server: UART pins unset in board_pins.h; UART disabled\n");
    }
    else
    {
        uart_init(s_uart, PICO_UART_BAUD);
        gpio_set_function((uint)PICO_UART_TX_GPIO, GPIO_FUNC_UART);
        gpio_set_function((uint)PICO_UART_RX_GPIO, GPIO_FUNC_UART);
        s_uart_ready = true;

        printf("uart_server: UART%u up @ %u baud (TX=%d RX=%d)\n",
               (unsigned)PICO_UART_ID,
               (unsigned)PICO_UART_BAUD,
               PICO_UART_TX_GPIO,
               PICO_UART_RX_GPIO);
    }

    init_drv8163();
}

void uart_server_poll(void)
{
    if (!s_uart_ready)
    {
        return;
    }

    while (uart_is_readable(s_uart))
    {
        uint8_t b = uart_getc(s_uart);
        if (!rx_ring_push(b))
        {
            s_frame_len = 0u;
            s_frame_overflow = true;
        }
    }

    uint8_t b = 0u;
    while (rx_ring_pop(&b))
    {
        if (b == PROTO_DELIM)
        {
            process_frame();
            continue;
        }

        if (s_frame_overflow)
        {
            continue;
        }

        if (s_frame_len >= sizeof(s_frame_buf))
        {
            s_frame_overflow = true;
            continue;
        }

        s_frame_buf[s_frame_len++] = b;
    }
}
