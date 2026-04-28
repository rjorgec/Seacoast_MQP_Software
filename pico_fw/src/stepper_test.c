#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

#include "board_pins.h"
#include "drivers/drv8434s/drv8434s.h"

/* Callback helpers (similar to uart_server.c) */
static bool s_spi_debug = false; /* enable with debugger or recompile */
static uint32_t s_spi_last_print_ms = 0u;

static int stepper_spi_xfer(void *ctx, const uint8_t *tx, uint8_t *rx, size_t len)
{
    spi_inst_t *spi = (spi_inst_t *)ctx;

    if (s_spi_debug)
    {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if ((now - s_spi_last_print_ms) >= 500u)
        {
            s_spi_last_print_ms = now;
            /* SPI debug: print bytes both directions */
            printf("SPI xfer len=%zu tx:", len);
            for (size_t i = 0; i < len; ++i)
            {
                printf(" %02X", tx[i]);
            }
            printf(" rx before:");
            for (size_t i = 0; i < len; ++i)
            {
                printf(" %02X", rx ? rx[i] : 0);
            }
            printf("\n");
        }
    }

    spi_write_read_blocking(spi, tx, rx, len);

    if (s_spi_debug)
    {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if ((now - s_spi_last_print_ms) >= 500u)
        {
            s_spi_last_print_ms = now;
            printf("SPI xfer rx after:");
            for (size_t i = 0; i < len; ++i)
            {
                printf(" %02X", rx[i]);
            }
            printf("\n");
        }
    }

    return 0;
}

static void stepper_cs(void *ctx, bool asserted)
{
    (void)ctx;
    /* DRV8434S CS is active-low: assert = drive low, deassert = drive high. */
    gpio_put((uint)DRV8434S_CS_GPIO, asserted ? 0u : 1u);
    if (s_spi_debug)
    {
        printf("CS %s\n", asserted ? "assert" : "deassert");
    }
}

static void stepper_delay_us(void *ctx, unsigned us)
{
    (void)ctx;
    sleep_us(us);
}

static void stepper_delay_ms(void *ctx, unsigned ms)
{
    (void)ctx;
    sleep_ms(ms);
}

/* globals mirroring uart_server structure */
static drv8434s_chain_t s_chain;
static drv8434s_motion_t s_motion;
static struct repeating_timer s_step_timer;
static bool s_step_timer_active;
static bool s_need_new_motion;
static bool s_reverse_next;
static uint32_t s_last_done_ms;

static void stepper_motion_done(void *ctx, uint8_t dev_idx,
                                const drv8434s_motion_result_t *res)
{
    (void)ctx;
    #ifdef STEP_DEBUG
    printf("Stepper[%u]: done — %li of %li steps, torque=%u, reason=%u\n",
           dev_idx,
           (long)res->steps_achieved,
           (long)res->steps_requested,
           res->last_torque_count,
           (unsigned)res->reason);
    #endif
    s_last_done_ms = to_ms_since_boot(get_absolute_time());
    s_need_new_motion = true;
    /* toggle direction for next job */
    s_reverse_next = !s_reverse_next;
}

static bool step_timer_callback(struct repeating_timer *t)
{
    (void)t;
    drv8434s_motion_tick(&s_motion);

    if (!drv8434s_motion_is_busy(&s_motion))
    {
        s_step_timer_active = false;
        return false;
    }
    return true;
}

static bool ensure_step_timer_running(uint32_t delay_us)
{
    if (s_step_timer_active)
        return true;
    if (!add_repeating_timer_us(-(int64_t)delay_us,
                                step_timer_callback, NULL, &s_step_timer))
    {
        return false;
    }
    s_step_timer_active = true;
    return true;
}

int main(void)
{
    stdio_init_all();
    sleep_ms(10000);
    printf("stepper_test: start\n");
    printf("Pins: SCK=%d MOSI=%d MISO=%d CS=%d\n",
           DRV8434S_SCK_GPIO,
           DRV8434S_MOSI_GPIO,
           DRV8434S_MISO_GPIO,
           DRV8434S_CS_GPIO);
    /* enable verbose SPI logging so we can see CS toggles */
    s_spi_debug = true;

    /* SPI setup */
    spi_inst_t *spi = (DRV8434S_SPI_ID == 0) ? spi0 : spi1;
    spi_init(spi, (uint)DRV8434S_SPI_BAUD);
    spi_set_format(spi, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);

    gpio_set_function((uint)DRV8434S_SCK_GPIO, GPIO_FUNC_SPI);
    gpio_set_function((uint)DRV8434S_MOSI_GPIO, GPIO_FUNC_SPI);
    gpio_set_function((uint)DRV8434S_MISO_GPIO, GPIO_FUNC_SPI);

    gpio_init((uint)DRV8434S_CS_GPIO);
    gpio_set_dir((uint)DRV8434S_CS_GPIO, GPIO_OUT);
    gpio_put((uint)DRV8434S_CS_GPIO, 1u);

    drv8434s_chain_config_t cfg = {
        .user_ctx = spi,
        .spi_xfer = stepper_spi_xfer,
        .cs = stepper_cs,
        .delay_ms = stepper_delay_ms,
        .delay_us = stepper_delay_us,
        .n_devices = (uint8_t)DRV8434S_N_DEVICES,
    };

    if (!drv8434s_chain_init(&s_chain, &cfg))
    {
        printf("stepper_test: chain init failed\n");
        return 1;
    }

    /* probe each device's FAULT register immediately to verify presence */
    bool any_ok = false;
    for (uint8_t k = 0; k < s_chain.cfg.n_devices; ++k)
    {
        uint8_t fault;
        if (drv8434s_chain_read_reg(&s_chain, k, DRV8434S_REG_FAULT, &fault, NULL))
        {
            printf("stepper_test: probe dev %u OK, FAULT=0x%02X\n", k, fault);
            if (fault != 0xFF)
                any_ok = true;
        }
        else
        {
            printf("stepper_test: probe dev %u FAILED\n", k);
        }
    }
    if (!any_ok)
    {
        printf("stepper_test: no devices responded – check SPI wiring/power\n");
        while (true)
            tight_loop_contents();
    }

    for (uint8_t k = 0; k < s_chain.cfg.n_devices; ++k)
    {
        if (!drv8434s_chain_set_spi_step_mode(&s_chain, k, NULL))
        {
            printf("stepper_test: set SPI step mode failed (dev %u)\n", k);
        }
        if (!drv8434s_chain_enable(&s_chain, k, NULL))
        {
            printf("stepper_test: enable outputs failed (dev %u)\n", k);
        }
    }

    if (!drv8434s_motion_init(&s_motion, &s_chain,
                              stepper_motion_done, NULL, 1u))
    {
        printf("stepper_test: motion engine init failed\n");
        return 2;
    }

    printf("stepper_test: chain ready, beginning agitation on device 0\n");

    s_need_new_motion = true;
    s_reverse_next = false;
    s_last_done_ms = to_ms_since_boot(get_absolute_time());

    /* main loop schedules forward/pause/reverse repeats */
    while (true)
    {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (s_need_new_motion && !drv8434s_motion_is_busy(&s_motion) &&
            (now - s_last_done_ms) >= 500u)
        {
            s_need_new_motion = false;
            int32_t steps = (int32_t)AGITATOR_KNEAD_STEPS;
            if (s_reverse_next)
                steps = -steps;
            if (!drv8434s_motion_start(&s_motion, 0, steps, 0))
            {
                printf("stepper_test: motion start failed\n");
            }
            else
            {
                printf("stepper_test: scheduled %s motion %ld steps\n",
                       s_reverse_next ? "reverse" : "forward",
                       (long)steps);
                if (!ensure_step_timer_running(AGITATOR_STEP_DELAY_US))
                {
                    printf("stepper_test: failed to start step timer\n");
                }
            }
        }
        tight_loop_contents();
    }
}
