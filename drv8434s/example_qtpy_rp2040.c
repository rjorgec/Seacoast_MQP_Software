/*
 Example usage for QT Py RP2040 (Raspberry Pi Pico SDK)
 Build this file with the Pico SDK environment and link with hardware_spi and hardware_gpio.

 This file is a self-contained example that implements the callback functions expected by drv8434s.
*/

#ifdef TEST_QTYPY_RP2040

#include "drv8434s.h"
#include <hardware/spi.h>
#include <hardware/gpio.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pico/stdlib.h>

// Adjust these pins to your wiring
#define QT_PY_SPI_PORT spi0
#define PIN_SCK 2
#define PIN_MOSI 3
#define PIN_MISO 4
#define PIN_CS 5
#define PIN_RST 6

static int qtpy_spi_xfer(void *user_ctx, const uint8_t *tx, uint8_t *rx, size_t len)
{
    // Pico SDK's spi_write_read_blocking is full-duplex
    if (!tx && !rx)
        return -1;
    if (tx && rx)
    {
        spi_write_read_blocking(QT_PY_SPI_PORT, tx, rx, len);
    }
    else if (tx)
    {
        spi_write_blocking(QT_PY_SPI_PORT, tx, len);
    }
    else
    {
        // read-only: send zeros
        memset(rx, 0, len);
        spi_read_blocking(QT_PY_SPI_PORT, 0x00, rx, len);
    }
    return 0;
}

static void qtpy_cs(void *user_ctx, bool asserted)
{
    gpio_put(PIN_CS, asserted ? 0 : 1);
}

static void qtpy_reset(void *user_ctx, bool asserted)
{
    gpio_put(PIN_RST, asserted ? 0 : 1);
}

static void qtpy_delay(void *user_ctx, unsigned ms)
{
    sleep_ms(ms);
}

int main(void)
{
    stdio_init_all();
    // Init SPI pins
    spi_init(QT_PY_SPI_PORT, 1000 * 1000); // 1 MHz for probing
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);

    // CS and reset as outputs
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);

    gpio_init(PIN_RST);
    gpio_set_dir(PIN_RST, GPIO_OUT);
    gpio_put(PIN_RST, 1);

    drv8434s_config_t cfg = {
        .user_ctx = NULL,
        .spi_xfer = qtpy_spi_xfer,
        .cs = qtpy_cs,
        .reset = qtpy_reset,
        .delay_ms = qtpy_delay,
    };

    drv8434s_t dev;
    if (!drv8434s_init(&dev, &cfg))
    {
        printf("drv8434s_init failed\n");
        return 1;
    }

    // Optional reset
    drv8434s_reset(&dev, 10);

    bool present = drv8434s_probe(&dev);
    printf("DRV8434S probe: %s\n", present ? "present" : "not detected");

    // In a real application you'd continue with register reads/writes here.

    return present ? 0 : 2;
}

#endif // TEST_QTYPY_RP2040
