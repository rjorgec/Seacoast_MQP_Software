#ifndef DRV8434S_H
#define DRV8434S_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C"
{
#endif

    // Simple, portable driver API for DRV8434S-like SPI motor drivers.
    // The API is intentionally minimal and hardware-agnostic: callers
    // supply small callback functions for SPI transfers, GPIO, and delay.

    typedef int (*drv8434s_spi_xfer_t)(void *user_ctx, const uint8_t *tx, uint8_t *rx, size_t len);
    typedef void (*drv8434s_cs_control_t)(void *user_ctx, bool asserted);
    typedef void (*drv8434s_reset_t)(void *user_ctx, bool asserted);
    typedef void (*drv8434s_delay_ms_t)(void *user_ctx, unsigned ms);

    typedef struct
    {
        void *user_ctx;               // passed back to callbacks
        drv8434s_spi_xfer_t spi_xfer; // full-duplex SPI transfer
        drv8434s_cs_control_t cs;     // assert/deassert chip-select (active low)
        drv8434s_reset_t reset;       // optional reset control (can be NULL)
        drv8434s_delay_ms_t delay_ms; // optional delay function (can be NULL)
    } drv8434s_config_t;

    typedef struct
    {
        drv8434s_config_t cfg;
    } drv8434s_t;

    // Initialize device context. Returns true on success.
    bool drv8434s_init(drv8434s_t *dev, const drv8434s_config_t *cfg);

    // Probe the device: performs a simple SPI transaction and returns true
    // if the device responded (i.e., MISO returned something other than 0xFF).
    // This is intentionally conservative and portable across platforms.
    bool drv8434s_probe(drv8434s_t *dev);

    // Reset the device using the reset callback (if provided).
    void drv8434s_reset(drv8434s_t *dev, unsigned reset_ms);

    // Advanced helpers ---------------------------------------------------------
    // Send an arbitrary SPI command (tx) and read rx_len bytes into rx. Returns
    // true if the transfer succeeded and the returned bytes are not all 0xFF
    // (a simple heuristic that helps detect a real device vs. a floating MISO).
    bool drv8434s_probe_cmd(drv8434s_t *dev, const uint8_t *tx, size_t tx_len, uint8_t *rx, size_t rx_len);

    // Read a register using a common [reg][dummy...]-style SPI protocol. This
    // helper sends `reg` followed by `out_len` dummy bytes and copies the
    // received bytes into `out`. Returns true if the transfer succeeded and at
    // least one returned byte is not 0xFF.
    bool drv8434s_read_register(drv8434s_t *dev, uint8_t reg, uint8_t *out, size_t out_len);

#ifdef __cplusplus
}
#endif
#endif // DRV8434S_H
