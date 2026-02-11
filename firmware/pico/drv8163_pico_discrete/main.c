#include "drv8163_pico.h"
#include "drv8434s.h"
#include "seacoast_proto.h"

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"

#include <stdio.h>
#include <string.h>

#ifndef DRV8434_CS_PIN
#define DRV8434_CS_PIN 13
#endif
#ifndef DRV8434_RST_PIN
#define DRV8434_RST_PIN 14
#endif
#ifndef DRV8434_STEP_PIN
#define DRV8434_STEP_PIN 15
#endif

#define PICO_LINK_UART uart0
#define PICO_LINK_TX_PIN 0
#define PICO_LINK_RX_PIN 1
#define PICO_LINK_BAUD 460800

#define DC_RUN_DUTY 12000
#define DC_CLEAN_DUTY 10000

#define STEP_INTERVAL_US 2000
#define STEP_PULSE_HIGH_US 80
#define MASS_PER_STEP_G 0.01f

#define STATUS_PERIOD_MS 100
#define CLEAN_PERIOD_MS 5000
#define HOME_PERIOD_MS 2500

typedef enum {
    PICO_STATE_IDLE = 0,
    PICO_STATE_RUNNING = 1,
    PICO_STATE_PAUSED = 2,
    PICO_STATE_CLEANING = 3,
    PICO_STATE_HOMING = 4,
    PICO_STATE_FAULT = 5,
} pico_state_t;

typedef struct {
    pico_state_t state;
    bool stepper_present;
    bool step_pin_high;
    float target_g;
    float mass_g;
    uint16_t fault;
    int64_t next_step_us;
    int64_t step_low_us;
    int64_t action_end_us;
} runtime_t;

static drv8163_handle_t g_drv8163;
static sc_parser_t g_parser;
static runtime_t g_rt;

static int drv8434_spi_xfer(void *user_ctx, const uint8_t *tx, uint8_t *rx, size_t len)
{
    (void)user_ctx;
    if (tx && rx) {
        spi_write_read_blocking(DRV8163_SPI_PORT, tx, rx, len);
    } else if (tx) {
        spi_write_blocking(DRV8163_SPI_PORT, tx, len);
    } else if (rx) {
        spi_read_blocking(DRV8163_SPI_PORT, 0x00, rx, len);
    } else {
        return -1;
    }
    return 0;
}

static void drv8434_cs(void *user_ctx, bool asserted)
{
    (void)user_ctx;
    gpio_put(DRV8434_CS_PIN, asserted ? 0 : 1);
}

static void drv8434_reset(void *user_ctx, bool asserted)
{
    (void)user_ctx;
    gpio_put(DRV8434_RST_PIN, asserted ? 0 : 1);
}

static uint8_t state_for_status(void)
{
    return (g_rt.state == PICO_STATE_RUNNING ||
            g_rt.state == PICO_STATE_CLEANING ||
            g_rt.state == PICO_STATE_HOMING) ? 1 : 0;
}

static void send_frame(uint8_t type, const void *payload, uint8_t len)
{
    uint8_t buf[2 + 2 + SC_MAX_PAYLOAD + 2];
    size_t n = sc_build_frame(type, payload, len, buf, sizeof(buf));
    if (n == 0) return;
    for (size_t i = 0; i < n; i++) {
        uart_putc_raw(PICO_LINK_UART, buf[i]);
    }
}

static void send_ack(uint8_t cmd, int8_t result)
{
    sc_ack_t ack = {
        .cmd_type = cmd,
        .result = result,
    };
    send_frame(SC_MSG_ACK, &ack, (uint8_t)sizeof(ack));
}

static void send_status(void)
{
    sc_status_t st = {
        .state = state_for_status(),
        .mass_g = g_rt.mass_g,
        .fault = g_rt.fault,
    };
    send_frame(SC_MSG_STATUS, &st, (uint8_t)sizeof(st));
}

static void stop_motion(void)
{
    drv8163_set_motor_control(MOTOR_STOP, 0);
    drv8163_set_motor_state(MOTOR_STOP);
    gpio_put(DRV8434_STEP_PIN, 0);
    g_rt.step_pin_high = false;
}

static void start_running(float target_g)
{
    if (target_g <= 0.0f) {
        target_g = 50.0f;
    }
    g_rt.target_g = target_g;
    g_rt.mass_g = 0.0f;
    g_rt.state = PICO_STATE_RUNNING;
    g_rt.next_step_us = to_us_since_boot(get_absolute_time()) + STEP_INTERVAL_US;
    g_rt.step_low_us = 0;
    drv8163_enable_driver();
    drv8163_set_motor_control(MOTOR_FORWARD, DC_RUN_DUTY);
}

static void run_clean_cycle(void)
{
    g_rt.state = PICO_STATE_CLEANING;
    g_rt.action_end_us = to_us_since_boot(get_absolute_time()) + (CLEAN_PERIOD_MS * 1000);
    drv8163_enable_driver();
    drv8163_set_motor_control(MOTOR_REVERSE, DC_CLEAN_DUTY);
    gpio_put(DRV8434_STEP_PIN, 0);
    g_rt.step_pin_high = false;
}

static void run_home_cycle(void)
{
    g_rt.state = PICO_STATE_HOMING;
    g_rt.action_end_us = to_us_since_boot(get_absolute_time()) + (HOME_PERIOD_MS * 1000);
    drv8163_enable_driver();
    drv8163_set_motor_control(MOTOR_REVERSE, DC_CLEAN_DUTY);
    gpio_put(DRV8434_STEP_PIN, 0);
    g_rt.step_pin_high = false;
}

static void check_drv8163_faults(void)
{
    uint8_t faults = 0;
    drv8163_status_t s = drv8163_read_faults(&g_drv8163, &faults);
    if (s == DRV8163_OK || s == DRV8163_ERR_FAULT) {
        if (faults != 0) {
            g_rt.fault = faults;
            g_rt.state = PICO_STATE_FAULT;
            stop_motion();
        }
    }
}

static void update_motion(int64_t now_us)
{
    if (g_rt.state == PICO_STATE_RUNNING) {
        if (g_rt.mass_g >= g_rt.target_g) {
            g_rt.state = PICO_STATE_IDLE;
            stop_motion();
            return;
        }

        if (g_rt.stepper_present) {
            if (g_rt.step_pin_high) {
                if (now_us >= g_rt.step_low_us) {
                    gpio_put(DRV8434_STEP_PIN, 0);
                    g_rt.step_pin_high = false;
                }
            } else if (now_us >= g_rt.next_step_us) {
                gpio_put(DRV8434_STEP_PIN, 1);
                g_rt.step_pin_high = true;
                g_rt.step_low_us = now_us + STEP_PULSE_HIGH_US;
                g_rt.next_step_us += STEP_INTERVAL_US;
                g_rt.mass_g += MASS_PER_STEP_G;
            }
        } else {
            g_rt.mass_g += 0.002f;
        }
    } else if (g_rt.state == PICO_STATE_CLEANING || g_rt.state == PICO_STATE_HOMING) {
        if (now_us >= g_rt.action_end_us) {
            g_rt.state = PICO_STATE_IDLE;
            stop_motion();
        }
    }
}

static void handle_frame(const sc_frame_view_t *f)
{
    int8_t result = 0;

    switch (f->type) {
        case SC_MSG_CMD_START: {
            if (f->len != sizeof(sc_cmd_start_t)) {
                result = -1;
                break;
            }
            sc_cmd_start_t p;
            memcpy(&p, f->payload, sizeof(p));
            start_running(p.target_g);
            break;
        }
        case SC_MSG_CMD_STOP:
            g_rt.state = PICO_STATE_IDLE;
            stop_motion();
            break;
        case SC_MSG_CMD_TARE:
            g_rt.mass_g = 0.0f;
            break;
        case SC_MSG_CMD_CLEAN:
            run_clean_cycle();
            break;
        case SC_MSG_CMD_HOME:
            run_home_cycle();
            break;
        case SC_MSG_CMD_PAUSE:
            g_rt.state = PICO_STATE_PAUSED;
            stop_motion();
            break;
        default:
            result = -2;
            break;
    }

    send_ack(f->type, result);
}

static void process_uart_rx(void)
{
    while (uart_is_readable(PICO_LINK_UART)) {
        uint8_t b = (uint8_t)uart_getc(PICO_LINK_UART);
        sc_frame_view_t f;
        if (sc_parser_feed(&g_parser, b, &f)) {
            handle_frame(&f);
        }
    }
}

int main(void)
{
    stdio_init_all();
    sleep_ms(50);

    uart_init(PICO_LINK_UART, PICO_LINK_BAUD);
    gpio_set_function(PICO_LINK_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(PICO_LINK_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(PICO_LINK_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(PICO_LINK_UART, true);

    drv8163_pico_spi_init(1000 * 1000);
    drv8163_pico_gpio_init();
    drv8163_pico_pwm_init(1000);
    drv8163_pico_itrip_pwm_init(1000);
    drv8163_pico_adc_init();

    drv8163_status_t st = drv8163_init(&g_drv8163,
                                       drv8163_pico_spi_transfer,
                                       drv8163_pico_cs_assert,
                                       drv8163_pico_cs_deassert);
    if (st == DRV8163_OK) {
        drv8163_set_trip_current(&g_drv8163, 1);
        drv8163_enable_driver();
    } else {
        g_rt.fault = 0x100;
        g_rt.state = PICO_STATE_FAULT;
    }

    gpio_init(DRV8434_CS_PIN);
    gpio_set_dir(DRV8434_CS_PIN, GPIO_OUT);
    gpio_put(DRV8434_CS_PIN, 1);
    gpio_init(DRV8434_RST_PIN);
    gpio_set_dir(DRV8434_RST_PIN, GPIO_OUT);
    gpio_put(DRV8434_RST_PIN, 1);
    gpio_init(DRV8434_STEP_PIN);
    gpio_set_dir(DRV8434_STEP_PIN, GPIO_OUT);
    gpio_put(DRV8434_STEP_PIN, 0);

    drv8434s_config_t dcfg = {
        .user_ctx = NULL,
        .spi_xfer = drv8434_spi_xfer,
        .cs = drv8434_cs,
        .reset = drv8434_reset,
        .delay_ms = (drv8434s_delay_ms_t)sleep_ms,
        .delay_us = NULL,
    };

    drv8434s_t stepdev;
    bool step_ok = drv8434s_init(&stepdev, &dcfg);
    if (step_ok) {
        drv8434s_reset(&stepdev, 10);
        g_rt.stepper_present = drv8434s_probe(&stepdev);
        if (g_rt.stepper_present) {
            drv8434s_set_torque(&stepdev, 2);
            drv8434s_enable(&stepdev);
        }
    }

    g_rt.state = PICO_STATE_IDLE;
    sc_parser_init(&g_parser);

    int64_t last_status_us = to_us_since_boot(get_absolute_time());
    int64_t last_fault_check_us = last_status_us;

    while (true) {
        int64_t now_us = to_us_since_boot(get_absolute_time());

        process_uart_rx();
        update_motion(now_us);

        if ((now_us - last_fault_check_us) >= 100000) {
            check_drv8163_faults();
            last_fault_check_us = now_us;
        }

        if ((now_us - last_status_us) >= (STATUS_PERIOD_MS * 1000)) {
            send_status();
            last_status_us = now_us;
        }

        sleep_ms(1);
    }

    return 0;
}
