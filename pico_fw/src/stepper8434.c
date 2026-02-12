#include "stepper8434s.h"
#include "board_pins.h"

#include "drv8434s.h"

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/time.h"

#include <stdio.h>
#include <string.h>

static drv8434s_t g_dev;
static volatile bool g_fault_latched = false;

typedef struct {
    bool active;
    bool dir;
    uint32_t remaining;
    uint32_t step_delay_us;
    absolute_time_t next_step;
} step_job_t;

static step_job_t g_job = {0};

// ---- drv8434s platform callbacks (from your demo) ----
static int pico_spi_xfer(void *user_ctx, const uint8_t *tx, uint8_t *rx, size_t len) {
    (void)user_ctx;

    if (len == 2 && tx && rx) {
        uint16_t tx16 = ((uint16_t)tx[0] << 8) | tx[1];
        uint16_t rx16 = 0;
        spi_write16_read16_blocking(DRV8434S_SPI_PORT, &tx16, &rx16, 1);
        rx[0] = (uint8_t)(rx16 >> 8);
        rx[1] = (uint8_t)(rx16 & 0xFF);
        return 0;
    }
    if (len == 2 && tx) {
        uint16_t tx16 = ((uint16_t)tx[0] << 8) | tx[1];
        spi_write16_blocking(DRV8434S_SPI_PORT, &tx16, 1);
        return 0;
    }

    // fallback (shouldn’t be needed)
    if (tx && rx) spi_write_read_blocking(DRV8434S_SPI_PORT, tx, rx, len);
    else if (tx) spi_write_blocking(DRV8434S_SPI_PORT, tx, len);
    else if (rx) spi_read_blocking(DRV8434S_SPI_PORT, 0x00, rx, len);
    else return -1;

    return 0;
}

static void pico_cs(void *user_ctx, bool asserted) {
    (void)user_ctx;
    gpio_put(DRV8434S_PIN_CS, asserted ? 0 : 1);
}
static void pico_reset(void *user_ctx, bool asserted) {
    (void)user_ctx;
    gpio_put(DRV8434S_PIN_RST, asserted ? 0 : 1);
}
static void pico_delay_ms(void *user_ctx, unsigned ms) {
    (void)user_ctx;
    sleep_ms(ms);
}
static void pico_delay_us(void *user_ctx, unsigned us) {
    (void)user_ctx;
    busy_wait_us_32(us);
}

static void nfault_isr(uint gpio, uint32_t events) {
    if (gpio == DRV8434S_PIN_NFAULT && (events & GPIO_IRQ_EDGE_FALL)) {
        g_fault_latched = true;
        g_job.active = false; // stop job ASAP
    }
}

static void spi_pins_init(void) {
    spi_init(DRV8434S_SPI_PORT, 1000 * 1000);
    spi_set_format(DRV8434S_SPI_PORT, 16, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);

    gpio_set_function(DRV8434S_PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(DRV8434S_PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(DRV8434S_PIN_MISO, GPIO_FUNC_SPI);

    gpio_init(DRV8434S_PIN_CS);
    gpio_set_dir(DRV8434S_PIN_CS, GPIO_OUT);
    gpio_put(DRV8434S_PIN_CS, 1);

    gpio_init(DRV8434S_PIN_RST);
    gpio_set_dir(DRV8434S_PIN_RST, GPIO_OUT);
    gpio_put(DRV8434S_PIN_RST, 1);

    gpio_init(DRV8434S_PIN_NFAULT);
    gpio_set_dir(DRV8434S_PIN_NFAULT, GPIO_IN);
    gpio_pull_up(DRV8434S_PIN_NFAULT);
}

bool stepper8434s_init(void) {
    spi_pins_init();

    drv8434s_config_t cfg = {
        .user_ctx = NULL,
        .spi_xfer = pico_spi_xfer,
        .cs = pico_cs,
        .reset = pico_reset,
        .delay_ms = pico_delay_ms,
        .delay_us = pico_delay_us,
    };

    if (!drv8434s_init(&g_dev, &cfg)) return false;

    drv8434s_reset(&g_dev, 10);

    // Note: probe will fail without hardware; that’s OK. We still want firmware to run.
    (void)drv8434s_probe(&g_dev);

    // Configure “reasonable defaults” (you can tune later)
    drv8434s_set_spi_step_mode(&g_dev);
    drv8434s_set_microstep(&g_dev, DRV8434S_STEP_1_128);
    drv8434s_set_torque(&g_dev, 0);
    drv8434s_set_decay(&g_dev, DRV8434S_DECAY_SMART_TUNE_RIPPLE);

    // Enable IRQ
    gpio_set_irq_enabled_with_callback(DRV8434S_PIN_NFAULT, GPIO_IRQ_EDGE_FALL, true, nfault_isr);

    g_fault_latched = false;
    g_job.active = false;

    return true;
}

bool stepper8434s_set_enabled(bool en) {
    if (en) return drv8434s_enable(&g_dev);
    return drv8434s_disable(&g_dev);
}

bool stepper8434s_start_job(bool dir, uint32_t steps, uint32_t step_delay_us) {
    if (steps == 0 || step_delay_us == 0) return false;

    g_fault_latched = false;
    drv8434s_set_spi_dir(&g_dev, dir);

    g_job.active = true;
    g_job.dir = dir;
    g_job.remaining = steps;
    g_job.step_delay_us = step_delay_us;
    g_job.next_step = delayed_by_us(get_absolute_time(), step_delay_us);

    return true;
}

void stepper8434s_stop(void) {
    g_job.active = false;
}

void stepper8434s_tick(void) {
    if (!g_job.active) return;
    if (g_fault_latched) { g_job.active = false; return; }

    if (absolute_time_diff_us(get_absolute_time(), g_job.next_step) > 0) return;

    (void)drv8434s_spi_step(&g_dev);

    if (--g_job.remaining == 0) {
        g_job.active = false;
        return;
    }

    g_job.next_step = delayed_by_us(g_job.next_step, g_job.step_delay_us);
}

bool stepper8434s_faulted(void) {
    return g_fault_latched;
}
