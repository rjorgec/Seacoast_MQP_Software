// Test main: runs a DC motor slowly with low trip current, probes a DRV8434S
// via SPI and toggles a step pin to step a motor. Adjust pins as needed.

#include "drv8163_pico.h"
#include "drv8434s.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include <stdio.h>

// Adjust these pins to match your wiring
#ifndef DRV8434_CS_PIN
#define DRV8434_CS_PIN 13
#endif
#ifndef DRV8434_RST_PIN
#define DRV8434_RST_PIN 14
#endif
#ifndef DRV8434_STEP_PIN
#define DRV8434_STEP_PIN 15
#endif

// Simple SPI transfer wrapper for drv8434s (matches drv8434s_spi_xfer_t)
static int drv8434_spi_xfer(void *user_ctx, const uint8_t *tx, uint8_t *rx, size_t len)
{
    // Use the same SPI port as DRV8163 (DRV8163_SPI_PORT)
    if (tx && rx)
    {
        spi_write_read_blocking(DRV8163_SPI_PORT, tx, rx, len);
    }
    else if (tx)
    {
        spi_write_blocking(DRV8163_SPI_PORT, tx, len);
    }
    else if (rx)
    {
        spi_read_blocking(DRV8163_SPI_PORT, 0x00, rx, len);
    }
    return 0;
}

static void drv8434_cs(void *user_ctx, bool asserted)
{
    gpio_put(DRV8434_CS_PIN, asserted ? 0 : 1);
}

static void drv8434_reset(void *user_ctx, bool asserted)
{
    gpio_put(DRV8434_RST_PIN, asserted ? 0 : 1);
}

int main(void)
{
    stdio_init_all();
    sleep_ms(50);
    printf("Starting DRV8163 + DRV8434 test\n");

    // Initialize SPI for DRV8163 and DRV8434 (shared SPI bus, separate CS)
    drv8163_pico_spi_init(1000 * 1000); // 1 MHz for safe probing

    // Prepare DRV8434 pins
    gpio_init(DRV8434_CS_PIN);
    gpio_set_dir(DRV8434_CS_PIN, GPIO_OUT);
    gpio_put(DRV8434_CS_PIN, 1);

    gpio_init(DRV8434_RST_PIN);
    gpio_set_dir(DRV8434_RST_PIN, GPIO_OUT);
    gpio_put(DRV8434_RST_PIN, 1);

    gpio_init(DRV8434_STEP_PIN);
    gpio_set_dir(DRV8434_STEP_PIN, GPIO_OUT);
    gpio_put(DRV8434_STEP_PIN, 0);

    // Initialize DRV8163 GPIO and PWM
    drv8163_pico_gpio_init();
    drv8163_pico_pwm_init(1000);       // 1 kHz PWM for motor
    drv8163_pico_itrip_pwm_init(1000); // 1 kHz for ITRIP (simulated DAC)
    drv8163_pico_adc_init();

    // Create and initialize driver handle for DRV8163
    drv8163_handle_t drv_handle;
    drv8163_status_t st = drv8163_init(&drv_handle, drv8163_pico_spi_transfer, drv8163_pico_cs_assert, drv8163_pico_cs_deassert);
    if (st != DRV8163_OK)
    {
        printf("DRV8163 init failed: %d\n", st);
    }
    else
    {
        // Configure low trip current (low amp limit)
        drv8163_set_trip_current(&drv_handle, 1); // small trip setting (adjust as needed)
    }

    // Set initial motor state and low PWM duty
    drv8163_set_motor_state(MOTOR_STOP);
    drv8163_set_pwm_duty_cycle(4096); // small duty (~6% of 65535)
    drv8163_enable_driver();

    // Initialize drv8434s device context using callbacks
    drv8434s_config_t dcfg = {
        .user_ctx = NULL,
        .spi_xfer = drv8434_spi_xfer,
        .cs = drv8434_cs,
        .reset = drv8434_reset,
        .delay_ms = (drv8434s_delay_ms_t)sleep_ms,
    };

    drv8434s_t stepdev;
    bool ok = drv8434s_init(&stepdev, &dcfg);
    printf("drv8434s_init: %s\n", ok ? "ok" : "fail");
    drv8434s_reset(&stepdev, 10);
    bool present = drv8434s_probe(&stepdev);
    printf("DRV8434 probe: %s\n", present ? "present" : "not detected");

    // Start motor slowly and step the stepper in the main loop
    drv8163_set_motor_state(MOTOR_FORWARD);

    while (true)
    {
        // slowly vary DC motor duty cycle for a simple run
        for (uint32_t d = 4096; d <= 12000; d += 1024)
        {
            drv8163_set_pwm_duty_cycle((uint16_t)d);
            sleep_ms(200);
        }
        sleep_ms(500);

        // Step the stepper (if present) by toggling the STEP pin
        if (present)
        {
            for (int s = 0; s < 200; ++s)
            {
                gpio_put(DRV8434_STEP_PIN, 1);
                sleep_ms(2);
                gpio_put(DRV8434_STEP_PIN, 0);
                sleep_ms(2);
            }
        }

        // Small read of current sense to exercise ADC
        uint16_t adc = drv8163_read_iprobi_raw_adc();
        printf("IPROPI ADC: %u\n", adc);

        // coast for a bit
        drv8163_set_motor_state(MOTOR_STOP);
        sleep_ms(1000);
        drv8163_set_motor_state(MOTOR_FORWARD);
    }

    return 0;
}
