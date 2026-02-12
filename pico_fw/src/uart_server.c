#include "uart_server.h"

#include "pico/stdlib.h"
#include "hardware/uart.h"

#include <stdio.h>
#include <string.h>

#include "shared/proto.h"
#include "shared/cobs.h"

// --- motor driver headers (adjust paths to tree) ---
#include "stepper8434s.h"
#include "drv8163_pico.h"   // for drv8163_start_current_monitor_task / stop...
// For DRV8434S, you’ll likely have your own init wrapper; step call shown as placeholder:
// #include "drv8434s.h"

#define UART_ID     uart0
#define UART_BAUD   115200
#define UART_TX_PIN 0
#define UART_RX_PIN 1

// Safety
#define COMM_TIMEOUT_MS 500

typedef struct {
    bool active;
    uint8_t dir;
    uint32_t remaining;
    uint32_t step_delay_us;
    absolute_time_t next_step;
} stepper_job_t;

static stepper_job_t g_job = {0};
static absolute_time_t g_last_good = {0};

static void uart_send_frame(uint8_t type, uint16_t seq, const void *payload, uint16_t len) {
    uint8_t raw[sizeof(proto_hdr_t) + PROTO_MAX_PAYLOAD + 2];
    proto_hdr_t h = {
        .version = PROTO_VERSION,
        .type = type,
        .seq = seq,
        .len = len
    };

    memcpy(raw, &h, sizeof(h));
    if (payload && len) memcpy(raw + sizeof(h), payload, len);

    uint16_t crc = proto_crc16_ccitt(raw, (uint32_t)(sizeof(h) + len));
    raw[sizeof(h) + len + 0] = (uint8_t)(crc & 0xFF);
    raw[sizeof(h) + len + 1] = (uint8_t)(crc >> 8);

    uint8_t enc[sizeof(raw) + 8];
    size_t enc_len = cobs_encode(raw, sizeof(h) + len + 2, enc);

    uart_write_blocking(UART_ID, enc, enc_len);
    uint8_t z = PROTO_DELIM;
    uart_write_blocking(UART_ID, &z, 1);
}

static void send_ack(uint16_t seq) {
    uart_send_frame(MSG_ACK, seq, NULL, 0);
}

static void send_nack(uint16_t seq, nack_code_t code) {
    uint8_t pl = (uint8_t)code;
    uart_send_frame(MSG_NACK, seq, &pl, 1);
}

static void motors_estop(void) {
    // DRV8163: stop monitor + disable driver
    drv8163_stop_current_monitor_task();
    drv8163_set_motor_control(MOTOR_STOP, 0);
    drv8163_disable_driver();

    // DRV8434S: TODO: disable stepper outputs
    // drv8434s_disable(&g_stepper);

    g_job.active = false;
}

static void stepper_service(void) {
    if (!g_job.active) return;

    // time to step?
    if (absolute_time_diff_us(get_absolute_time(), g_job.next_step) > 0) return;

    // TODO: set direction if driver needs it
    // drv8434s_set_spi_dir(&g_stepper, g_job.dir == 1);

    // TODO: actual step command
    // drv8434s_spi_step(&g_stepper);

    g_job.remaining--;
    if (g_job.remaining == 0) {
        g_job.active = false;
        return;
    }
    g_job.next_step = delayed_by_us(g_job.next_step, g_job.step_delay_us);
}

static void dispatch_msg(const proto_hdr_t *h, const uint8_t *pl) {
    // we got a valid frame
    g_last_good = get_absolute_time();

    switch (h->type) {
        case MSG_PING:
            send_ack(h->seq);
            break;

        case MSG_DRV8163_START_MON: {
            if (h->len != sizeof(pl_drv8163_start_mon_t)) {
                send_nack(h->seq, NACK_BAD_LEN);
                break;
            }
            const pl_drv8163_start_mon_t *p = (const pl_drv8163_start_mon_t*)pl;

            // ensure driver enabled before monitor starts
            drv8163_enable_driver();
            bool ok = drv8163_start_current_monitor_task(
                p->speed, p->low_th, p->high_th, p->interval_ms
            );

            if (ok) send_ack(h->seq);
            else    send_nack(h->seq, NACK_UNKNOWN);
            break;
        }

        case MSG_DRV8163_STOP_MON:
            drv8163_stop_current_monitor_task();
            send_ack(h->seq);
            break;

            case MSG_STEPPER_ENABLE: {
                if (h->len != sizeof(pl_stepper_enable_t)) { send_nack(h->seq, NACK_BAD_LEN); break; }
                const pl_stepper_enable_t *p = (const pl_stepper_enable_t*)pl;
                bool ok = stepper8434s_set_enabled(p->enable != 0);
                if (ok) send_ack(h->seq); else send_nack(h->seq, NACK_UNKNOWN);
                break;
            }

            case MSG_STEPPER_STEPJOB: {
                if (h->len != sizeof(pl_stepper_stepjob_t)) { send_nack(h->seq, NACK_BAD_LEN); break; }
                const pl_stepper_stepjob_t *p = (const pl_stepper_stepjob_t*)pl;
                bool ok = stepper8434s_start_job(p->dir != 0, p->steps, p->step_delay_us);
                if (ok) send_ack(h->seq); else send_nack(h->seq, NACK_UNKNOWN);
                break;
            }

        default:
            send_nack(h->seq, NACK_UNKNOWN);
            break;
    }
}

void uart_server_init(void) {
    uart_init(UART_ID, UART_BAUD);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    g_last_good = get_absolute_time();

    // Optional: quick log
    printf("uart_server: up on uart0 %d baud, TX=%d RX=%d\n", UART_BAUD, UART_TX_PIN, UART_RX_PIN);

    // TODO: init your motor drivers here (DRV8163 SPI/GPIO/PWM/ADC, DRV8434S SPI, etc.)
}

void uart_server_poll(void) {
    static uint8_t frame[256];
    static size_t frame_len = 0;

    // Read bytes
    while (uart_is_readable(UART_ID)) {
        uint8_t b = uart_getc(UART_ID);

        if (b == PROTO_DELIM) {
            if (frame_len > 0) {
                // Try to decode COBS+CRC
                uint8_t dec[256];
                size_t dec_len = cobs_decode(frame, frame_len, dec);

                if (dec_len >= sizeof(proto_hdr_t) + 2) {
                    uint16_t got = (uint16_t)dec[dec_len-2] | ((uint16_t)dec[dec_len-1] << 8);
                    uint16_t calc = proto_crc16_ccitt(dec, (uint32_t)dec_len - 2);

                    if (got == calc) {
                        proto_hdr_t h;
                        memcpy(&h, dec, sizeof(h));

                        if (h.version != PROTO_VERSION) {
                            send_nack(h.seq, NACK_BAD_VERSION);
                        } else if (h.len > PROTO_MAX_PAYLOAD) {
                            send_nack(h.seq, NACK_BAD_LEN);
                        } else if (sizeof(proto_hdr_t) + h.len + 2 != dec_len) {
                            send_nack(h.seq, NACK_BAD_LEN);
                        } else {
                            const uint8_t *pl = dec + sizeof(proto_hdr_t);
                            dispatch_msg(&h, pl);
                        }
                    } else {
                        // CRC fail: ignore or NACK if you can parse seq (we can’t safely)
                    }
                }
            }
            frame_len = 0;
        } else {
            if (frame_len < sizeof(frame)) frame[frame_len++] = b;
            else frame_len = 0; // drop overflow
        }
    }

    // Service any non-blocking motor jobs
    stepper_service();

    // Watchdog: if comms stale, stop motors
    int64_t dt = absolute_time_diff_us(g_last_good, get_absolute_time());
    if (dt > (int64_t)COMM_TIMEOUT_MS * 1000) {
        motors_estop();
    }
}
