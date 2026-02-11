/*
  DRV8434S Arduino sketch for Adafruit QT Py (SAMD21)
  - Drop this folder into Arduino sketches or point arduino-cli at the folder to build.
  - Adjust PIN_CS and PIN_RST to match your wiring.
*/

#include <Arduino.h>
#include <SPI.h>
#include "drv8434s.h"

// Defaults; change to match your wiring
#ifndef PIN_CS
#define PIN_CS 7
#endif
#ifndef PIN_RST
#define PIN_RST 5
#endif

static int samd_spi_xfer(void *user_ctx, const uint8_t *tx, uint8_t *rx, size_t len)
{
    if (tx && rx)
    {
        for (size_t i = 0; i < len; ++i)
        {
            rx[i] = SPI.transfer(tx[i]);
        }
    }
    else if (tx)
    {
        for (size_t i = 0; i < len; ++i)
        {
            SPI.transfer(tx[i]);
        }
    }
    else if (rx)
    {
        for (size_t i = 0; i < len; ++i)
        {
            rx[i] = SPI.transfer(0x00);
        }
    }
    else
    {
        return -1;
    }
    return 0;
}

static void samd_cs(void *user_ctx, bool asserted)
{
    digitalWrite(PIN_CS, asserted ? LOW : HIGH); // active low
}

static void samd_reset(void *user_ctx, bool asserted)
{
    digitalWrite(PIN_RST, asserted ? LOW : HIGH);
}

static void samd_delay(void *user_ctx, unsigned ms)
{
    delay(ms);
}

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 2000)
    { /* wait for USB serial */
    }

    SPI.begin(); // default SPI pins for board

    pinMode(PIN_CS, OUTPUT);
    digitalWrite(PIN_CS, HIGH);

    pinMode(PIN_RST, OUTPUT);
    digitalWrite(PIN_RST, HIGH);

    drv8434s_config_t cfg = {
        .user_ctx = nullptr,
        .spi_xfer = samd_spi_xfer,
        .cs = samd_cs,
        .reset = samd_reset,
        .delay_ms = samd_delay,
    };

    drv8434s_t dev;
    if (!drv8434s_init(&dev, &cfg))
    {
        Serial.println("drv8434s_init failed");
        while (1)
            delay(1000);
    }

    drv8434s_reset(&dev, 10);

    bool present = drv8434s_probe(&dev);
    Serial.print("DRV8434S probe: ");
    Serial.println(present ? "present" : "not detected");

    // Try a more specific register read (common [reg][dummy...] SPI pattern)
    uint8_t regbuf[2] = {0xFF, 0xFF};
    if (drv8434s_read_register(&dev, 0x01, regbuf, sizeof(regbuf)))
    {
        char dbuf[64];
        sprintf(dbuf, "Read reg 0x01: 0x%02X 0x%02X", regbuf[0], regbuf[1]);
        Serial.println(dbuf);
    }
    else
    {
        Serial.println("Read register failed or returned 0xFF");
    }
}

void loop()
{
    delay(1000);
}
