#include "drv8434s.h"
#include <string.h>
#include <stdlib.h>

bool drv8434s_init(drv8434s_t *dev, const drv8434s_config_t *cfg)
{
    if (!dev || !cfg || !cfg->spi_xfer || !cfg->cs)
    {
        return false;
    }
    memset(dev, 0, sizeof(*dev));
    dev->cfg = *cfg;
    return true;
}

void drv8434s_reset(drv8434s_t *dev, unsigned reset_ms)
{
    if (!dev)
        return;
    if (dev->cfg.reset)
    {
        dev->cfg.reset(dev->cfg.user_ctx, true);
        if (dev->cfg.delay_ms)
            dev->cfg.delay_ms(dev->cfg.user_ctx, reset_ms);
        dev->cfg.reset(dev->cfg.user_ctx, false);
        if (dev->cfg.delay_ms)
            dev->cfg.delay_ms(dev->cfg.user_ctx, 10);
    }
}

bool drv8434s_probe(drv8434s_t *dev)
{
    if (!dev || !dev->cfg.spi_xfer || !dev->cfg.cs)
        return false;

    uint8_t tx[2] = {0x00, 0x00};
    uint8_t rx[2] = {0xFF, 0xFF};

    // assert CS (active low)
    dev->cfg.cs(dev->cfg.user_ctx, true);
    int r = dev->cfg.spi_xfer(dev->cfg.user_ctx, tx, rx, sizeof(tx));
    // deassert CS
    dev->cfg.cs(dev->cfg.user_ctx, false);

    (void)r; // platform-specific transfer return value ignored here

    // If the device is present, MISO should not be floating 0xFF for both bytes
    // (this is a heuristic probe and not a replacement for a proper register read)
    if (rx[0] != 0xFF || rx[1] != 0xFF)
    {
        return true;
    }
    return false;
}

bool drv8434s_probe_cmd(drv8434s_t *dev, const uint8_t *tx, size_t tx_len, uint8_t *rx, size_t rx_len)
{
    if (!dev || !dev->cfg.spi_xfer || !dev->cfg.cs)
        return false;
    if (rx_len == 0)
        return false;

    size_t len = tx_len > rx_len ? tx_len : rx_len;
    uint8_t *txbuf = (uint8_t *)calloc(len, 1);
    uint8_t *rxbuf = (uint8_t *)malloc(len);
    if (!txbuf || !rxbuf)
    {
        free(txbuf);
        free(rxbuf);
        return false;
    }

    if (tx && tx_len)
        memcpy(txbuf, tx, tx_len);

    dev->cfg.cs(dev->cfg.user_ctx, true);
    dev->cfg.spi_xfer(dev->cfg.user_ctx, txbuf, rxbuf, len);
    dev->cfg.cs(dev->cfg.user_ctx, false);

    // copy last rx_len bytes into rx (right-aligned)
    size_t start = len - rx_len;
    for (size_t i = 0; i < rx_len; ++i)
    {
        rx[i] = rxbuf[start + i];
    }

    // simple heuristic: if any returned byte is not 0xFF, consider device present
    bool any_valid = false;
    for (size_t i = 0; i < rx_len; ++i)
    {
        if (rx[i] != 0xFF)
        {
            any_valid = true;
            break;
        }
    }

    free(txbuf);
    free(rxbuf);
    return any_valid;
}

bool drv8434s_read_register(drv8434s_t *dev, uint8_t reg, uint8_t *out, size_t out_len)
{
    if (!dev || !out || out_len == 0)
        return false;
    // send register byte followed by dummy bytes to clock out response
    size_t tx_len = 1 + out_len;
    uint8_t *tx = (uint8_t *)calloc(tx_len, 1);
    uint8_t *rx = (uint8_t *)malloc(tx_len);
    if (!tx || !rx)
    {
        free(tx);
        free(rx);
        return false;
    }

    tx[0] = reg; // common convention: first byte is register address

    dev->cfg.cs(dev->cfg.user_ctx, true);
    dev->cfg.spi_xfer(dev->cfg.user_ctx, tx, rx, tx_len);
    dev->cfg.cs(dev->cfg.user_ctx, false);

    // copy received payload (skip the initial byte which is usually status/unknown)
    for (size_t i = 0; i < out_len; ++i)
    {
        out[i] = rx[1 + i];
    }

    // heuristic: success if any byte != 0xFF
    bool any_valid = false;
    for (size_t i = 0; i < out_len; ++i)
    {
        if (out[i] != 0xFF)
        {
            any_valid = true;
            break;
        }
    }

    free(tx);
    free(rx);
    return any_valid;
}
