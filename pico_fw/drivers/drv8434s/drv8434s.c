#include "drv8434s.h"
#include <string.h>
#include <stdlib.h>

// ── Internal timing helpers ─────────────────────────────────────────────────
//
// The DRV8434S datasheet specifies minimum CS-high hold time (t_CSH) between
// SPI frames, and the SPI_STEP bit requires a clean 0→1 rising edge with
// sufficient setup time.  Without these guards, register reads/writes may
// succeed but step commands can be silently ignored.

// Minimum CS-high hold time in microseconds between SPI frames.
// The datasheet value is ~50 ns, but real PCB layouts with shared SPI buses
// benefit from a more conservative guard.  Adjust to taste.
#define DRV8434S_CS_HOLD_US 5

// Settling time in microseconds after writing a register before issuing
// another SPI frame.  Ensures the internal state machine commits the write.
#define DRV8434S_REG_SETTLE_US 2

// Minimum time in microseconds between clearing and setting SPI_STEP to
// guarantee the device sees a rising edge.
#define DRV8434S_STEP_EDGE_US 5

static inline void drv_delay_us(drv8434s_t *dev, unsigned us)
{
    if (dev->cfg.delay_us)
    {
        dev->cfg.delay_us(dev->cfg.user_ctx, us);
    }
    else if (dev->cfg.delay_ms && us >= 1000)
    {
        dev->cfg.delay_ms(dev->cfg.user_ctx, us / 1000);
    }
    // If neither callback is provided, we fall through with no delay.
    // Callers on fast MCUs should always provide delay_us.
}

// Perform a single 2-byte SPI frame with proper CS assertion and hold timing.
static int drv_spi_frame(drv8434s_t *dev, const uint8_t tx[2], uint8_t rx[2])
{
    dev->cfg.cs(dev->cfg.user_ctx, true);
    int r = dev->cfg.spi_xfer(dev->cfg.user_ctx, tx, rx, 2);
    dev->cfg.cs(dev->cfg.user_ctx, false);

    // CS-high hold time: keep CS deasserted for at least t_CSH before the
    // next transaction.  This is critical on shared SPI buses and when
    // issuing rapid register writes (e.g. step commands).
    drv_delay_us(dev, DRV8434S_CS_HOLD_US);

    return r;
}

// ═════════════════════════════════════════════════════════════════════════════
//  Minimal API (unchanged signatures, improved timing)
// ═════════════════════════════════════════════════════════════════════════════

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

    // Send a read of register 0x00 (FAULT) — a proper DRV8434S command.
    // Using a real read command instead of bare zeros lets us distinguish
    // between a genuine device response and SPI artefacts.
    //
    // Expected SDO from a real DRV8434S:
    //   Byte 0 = FAULT register echoed as status (device-specific, ≠ 0xFF)
    //   Byte 1 = FAULT register data
    //
    // Loopback echo (no device): rx == tx exactly  → detected & rejected
    // Floating MISO (no device): rx == {0xFF,0xFF} → rejected
    uint8_t tx[2] = {DRV8434S_SPI_READ_BIT | ((DRV8434S_REG_FAULT & DRV8434S_ADDR_MASK) << DRV8434S_ADDR_SHIFT), 0x00};
    uint8_t rx[2] = {0xFF, 0xFF};

    int r = drv_spi_frame(dev, tx, rx);
    (void)r;

    // Reject loopback echo: if rx is identical to tx, MISO is shorted to
    // MOSI or the SPI frame format is wrong (inter-byte gap causing echo).
    if (rx[0] == tx[0] && rx[1] == tx[1])
    {
        return false;
    }

    // Reject floating MISO (all 0xFF = no device driving the line)
    if (rx[0] == 0xFF && rx[1] == 0xFF)
    {
        return false;
    }

    return true;
}

bool drv8434s_probe_cmd(drv8434s_t *dev, const uint8_t *tx, size_t tx_len,
                        uint8_t *rx, size_t rx_len)
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
    drv_delay_us(dev, DRV8434S_CS_HOLD_US);

    size_t start = len - rx_len;
    for (size_t i = 0; i < rx_len; ++i)
    {
        rx[i] = rxbuf[start + i];
    }

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

// ═════════════════════════════════════════════════════════════════════════════
//  Register-level API (with timing guards)
// ═════════════════════════════════════════════════════════════════════════════

bool drv8434s_write_reg(drv8434s_t *dev, uint8_t reg, uint8_t value)
{
    if (!dev || !dev->cfg.spi_xfer || !dev->cfg.cs)
        return false;

    // DRV8434S write frame: [0][0][A4:A0][0] [D7:D0]  (bit 14 = 0 → write)
    uint8_t tx[2] = {DRV8434S_SPI_WRITE | ((reg & DRV8434S_ADDR_MASK) << DRV8434S_ADDR_SHIFT), value};
    uint8_t rx[2] = {0};

    int r = drv_spi_frame(dev, tx, rx);

    // Post-write settling: give the device time to commit the register value
    // before the next SPI transaction.
    drv_delay_us(dev, DRV8434S_REG_SETTLE_US);

    if (r == 0 && (reg & DRV8434S_ADDR_MASK) < DRV8434S_NUM_REGS)
    {
        dev->reg_cache[reg & DRV8434S_ADDR_MASK] = value;
    }
    return (r == 0);
}

bool drv8434s_read_reg(drv8434s_t *dev, uint8_t reg, uint8_t *value)
{
    if (!dev || !value || !dev->cfg.spi_xfer || !dev->cfg.cs)
        return false;

    // DRV8434S read frame: [0][1][A4:A0][0] [0x00]  (bit 14 = 1 → read)
    uint8_t tx[2] = {DRV8434S_SPI_READ_BIT | ((reg & DRV8434S_ADDR_MASK) << DRV8434S_ADDR_SHIFT), 0x00};
    uint8_t rx[2] = {0xFF, 0xFF};

    int r = drv_spi_frame(dev, tx, rx);

    *value = rx[1]; // Data returned in the second byte

    if (r == 0 && (reg & DRV8434S_ADDR_MASK) < DRV8434S_NUM_REGS)
    {
        dev->reg_cache[reg & DRV8434S_ADDR_MASK] = rx[1];
    }
    return (r == 0);
}

bool drv8434s_modify_reg(drv8434s_t *dev, uint8_t reg, uint8_t mask,
                         uint8_t value)
{
    uint8_t current;
    if (!drv8434s_read_reg(dev, reg, &current))
        return false;

    uint8_t updated = (current & ~mask) | (value & mask);
    return drv8434s_write_reg(dev, reg, updated);
}

// ═════════════════════════════════════════════════════════════════════════════
//  Motor-control helpers
// ═════════════════════════════════════════════════════════════════════════════

bool drv8434s_enable(drv8434s_t *dev)
{
    // EN_OUT is CTRL2 bit 7
    return drv8434s_modify_reg(dev, DRV8434S_REG_CTRL2,
                               DRV8434S_CTRL2_EN_OUT,
                               DRV8434S_CTRL2_EN_OUT);
}

bool drv8434s_disable(drv8434s_t *dev)
{
    // EN_OUT is CTRL2 bit 7
    return drv8434s_modify_reg(dev, DRV8434S_REG_CTRL2,
                               DRV8434S_CTRL2_EN_OUT, 0);
}

bool drv8434s_set_microstep(drv8434s_t *dev, drv8434s_microstep_t mode)
{
    if ((uint8_t)mode > 9)
        return false;
    // MICROSTEP_MODE is CTRL3 bits [3:0]
    return drv8434s_modify_reg(dev, DRV8434S_REG_CTRL3,
                               DRV8434S_CTRL3_MICROSTEP_MASK,
                               (uint8_t)mode << DRV8434S_CTRL3_MICROSTEP_SHIFT);
}

bool drv8434s_set_torque(drv8434s_t *dev, uint8_t trq)
{
    if (trq > 15)
        trq = 15;
    // TRQ_DAC is CTRL1 bits [6:3]
    return drv8434s_modify_reg(dev, DRV8434S_REG_CTRL1,
                               DRV8434S_CTRL1_TRQ_DAC_MASK,
                               trq << DRV8434S_CTRL1_TRQ_DAC_SHIFT);
}

bool drv8434s_set_decay(drv8434s_t *dev, drv8434s_decay_t decay)
{
    // DECAY is CTRL2 bits [2:0]
    return drv8434s_modify_reg(dev, DRV8434S_REG_CTRL2,
                               DRV8434S_CTRL2_DECAY_MASK,
                               ((uint8_t)decay << DRV8434S_CTRL2_DECAY_SHIFT));
}

bool drv8434s_set_spi_step_mode(drv8434s_t *dev)
{
    // Set SPI_STEP (bit 4) and SPI_DIR (bit 5) to enable SPI control.
    // Once set, STEP (bit 6) and DIR (bit 7) become the active controls.
    return drv8434s_modify_reg(dev, DRV8434S_REG_CTRL3,
                               DRV8434S_CTRL3_SPI_STEP | DRV8434S_CTRL3_SPI_DIR,
                               DRV8434S_CTRL3_SPI_STEP | DRV8434S_CTRL3_SPI_DIR);
}

bool drv8434s_set_spi_dir(drv8434s_t *dev, bool reverse)
{
    // DIR is CTRL3 bit 7 (active when SPI_DIR mode is enabled)
    return drv8434s_modify_reg(dev, DRV8434S_REG_CTRL3,
                               DRV8434S_CTRL3_DIR,
                               reverse ? DRV8434S_CTRL3_DIR : 0);
}

bool drv8434s_spi_step(drv8434s_t *dev)
{
    if (!dev)
        return false;

    // The DRV8434S advances one step on the *rising edge* of the STEP
    // bit (CTRL3 bit 6, active when SPI_STEP mode is enabled).
    // The bit is self-clearing on the device, but drv8434s_write_reg()
    // caches the written value.  If we don't correct the cache, the next
    // call reads STEP=1 from the cache, writes the same value, and the
    // device sees no edge → no step.
    //
    // Fix: strip STEP from the base value before OR-ing, and restore
    // the cache to the base value (STEP=0) after the write so the next
    // call always produces a clean 0→1 transition.

    uint8_t addr = DRV8434S_REG_CTRL3 & DRV8434S_ADDR_MASK;
    uint8_t ctrl3_base = dev->reg_cache[addr] & (uint8_t)~DRV8434S_CTRL3_STEP;

    uint8_t ctrl3_high = ctrl3_base | DRV8434S_CTRL3_STEP;
    if (!drv8434s_write_reg(dev, DRV8434S_REG_CTRL3, ctrl3_high))
        return false;

    // STEP is self-clearing on the device — update the shadow cache
    // so the next call sees STEP=0 and can produce a rising edge.
    dev->reg_cache[addr] = ctrl3_base;

    return true;
}

// ═════════════════════════════════════════════════════════════════════════════
//  Stall detection helpers
// ═════════════════════════════════════════════════════════════════════════════

bool drv8434s_enable_stall_detection(drv8434s_t *dev)
{
    // Set both EN_STL (enable stall comparator) and STL_REP (report on nFAULT)
    return drv8434s_modify_reg(dev, DRV8434S_REG_CTRL5,
                               DRV8434S_CTRL5_EN_STL | DRV8434S_CTRL5_STL_REP,
                               DRV8434S_CTRL5_EN_STL | DRV8434S_CTRL5_STL_REP);
}

bool drv8434s_disable_stall_detection(drv8434s_t *dev)
{
    return drv8434s_modify_reg(dev, DRV8434S_REG_CTRL5,
                               DRV8434S_CTRL5_EN_STL, 0);
}

bool drv8434s_set_stall_threshold(drv8434s_t *dev, uint16_t threshold)
{
    if (threshold > 0x0FFF)
        threshold = 0x0FFF; // 12-bit max

    // Low 8 bits → CTRL6 (STALL_TH[7:0])
    uint8_t low = (uint8_t)(threshold & 0xFF);
    if (!drv8434s_write_reg(dev, DRV8434S_REG_CTRL6, low))
        return false;

    // High 4 bits → CTRL7 bits [3:0] (STALL_TH[11:8])
    return drv8434s_modify_reg(dev, DRV8434S_REG_CTRL7,
                               DRV8434S_CTRL7_STALL_TH_HI_MASK,
                               (uint8_t)((threshold >> 8) & 0x0F));
}

bool drv8434s_start_stall_learning(drv8434s_t *dev)
{
    // Set STL_LRN bit in CTRL5 to start the learning process.
    // The device will briefly stall the motor and update STALL_TH
    // automatically.  STL_LRN is self-clearing.
    return drv8434s_modify_reg(dev, DRV8434S_REG_CTRL5,
                               DRV8434S_CTRL5_STL_LRN,
                               DRV8434S_CTRL5_STL_LRN);
}

bool drv8434s_is_stall_learned(drv8434s_t *dev)
{
    if (!dev)
        return false;
    uint8_t diag2 = 0;
    if (!drv8434s_read_reg(dev, DRV8434S_REG_DIAG2, &diag2))
        return false;
    return (diag2 & DRV8434S_DIAG2_STL_LRN_OK) != 0;
}

// ═════════════════════════════════════════════════════════════════════════════
//  Diagnostics & fault helpers
// ═════════════════════════════════════════════════════════════════════════════

bool drv8434s_read_fault(drv8434s_t *dev, uint8_t *faults)
{
    return drv8434s_read_reg(dev, DRV8434S_REG_FAULT, faults);
}

bool drv8434s_clear_faults(drv8434s_t *dev)
{
    // CLR_FLT is CTRL4 bit 7
    return drv8434s_modify_reg(dev, DRV8434S_REG_CTRL4,
                               DRV8434S_CTRL4_CLR_FLT,
                               DRV8434S_CTRL4_CLR_FLT);
}

bool drv8434s_read_torque_count(drv8434s_t *dev, uint8_t *trq_count)
{
    // TRQ_COUNT [7:0] is in CTRL8 (0x0A)
    return drv8434s_read_reg(dev, DRV8434S_REG_CTRL8, trq_count);
}

bool drv8434s_read_all_regs(drv8434s_t *dev, uint8_t regs[DRV8434S_NUM_REGS])
{
    if (!dev || !regs)
        return false;

    for (uint8_t r = 0; r < DRV8434S_NUM_REGS; ++r)
    {
        if (!drv8434s_read_reg(dev, r, &regs[r]))
            return false;
    }
    return true;
}

bool drv8434s_diagnose_fault(drv8434s_t *dev, drv8434s_fault_info_t *info)
{
    if (!dev || !info)
        return false;

    memset(info, 0, sizeof(*info));

    if (!drv8434s_read_reg(dev, DRV8434S_REG_FAULT, &info->fault))
        return false;
    if (!drv8434s_read_reg(dev, DRV8434S_REG_DIAG1, &info->diag1))
        return false;
    if (!drv8434s_read_reg(dev, DRV8434S_REG_DIAG2, &info->diag2))
        return false;

    info->is_faulted = (info->fault & DRV8434S_FAULT_FAULT) != 0;
    return true;
}

drv8434s_recover_result_t drv8434s_recover_from_fault(
    drv8434s_t *dev, drv8434s_fault_info_t *info, unsigned reset_ms)
{
    if (!dev || !info)
        return DRV8434S_RECOVER_SPI_ERROR;

    // ── Stage 0: capture pre-recovery fault snapshot ────────────────────
    if (!drv8434s_diagnose_fault(dev, info))
        return DRV8434S_RECOVER_SPI_ERROR;

    // No fault active — nothing to do.
    if (!info->is_faulted)
        return DRV8434S_RECOVER_OK;

    // ── Stage 1: software clear (CLR_FLT bit in CTRL4) ─────────────────
    if (!drv8434s_clear_faults(dev))
        return DRV8434S_RECOVER_SPI_ERROR;

    // Small delay for the device to process the clear.
    if (dev->cfg.delay_ms)
        dev->cfg.delay_ms(dev->cfg.user_ctx, 5);

    // Re-read fault state after software clear.
    if (!drv8434s_diagnose_fault(dev, info))
        return DRV8434S_RECOVER_SPI_ERROR;

    if (!info->is_faulted)
        return DRV8434S_RECOVER_CLR_OK;

    // ── Stage 2: hardware reset (nRST toggle) if callback available ────
    if (dev->cfg.reset)
    {
        // Save writable register contents from the shadow cache so we can
        // restore them after the reset wipes all registers to defaults.
        // CTRL1 (0x03) through CTRL7 (0x09) are the writable registers.
        uint8_t saved_ctrl[7]; // CTRL1 .. CTRL7  (addresses 0x03 – 0x09)
        for (uint8_t r = 0; r < 7; ++r)
            saved_ctrl[r] = dev->reg_cache[DRV8434S_REG_CTRL1 + r];

        drv8434s_reset(dev, reset_ms > 0 ? reset_ms : 10);

        // Restore saved configuration registers.
        // CTRL4 (0x06) contains CLR_FLT; we mask it out to avoid
        // inadvertently re-writing a 1 to the self-clearing bit.
        for (uint8_t r = 0; r < 7; ++r)
        {
            uint8_t addr = DRV8434S_REG_CTRL1 + r;
            uint8_t val = saved_ctrl[r];

            // Mask out the self-clearing CLR_FLT bit in CTRL4
            if (addr == DRV8434S_REG_CTRL4)
                val &= (uint8_t)~DRV8434S_CTRL4_CLR_FLT;

            drv8434s_write_reg(dev, addr, val);
        }

        // Re-read fault state after hardware reset + restore.
        if (!drv8434s_diagnose_fault(dev, info))
            return DRV8434S_RECOVER_SPI_ERROR;

        if (!info->is_faulted)
            return DRV8434S_RECOVER_RESET_OK;
    }

    // Faults persist after all recovery attempts.
    return DRV8434S_RECOVER_STILL_FAULTED;
}