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

bool drv8434s_set_fault_config(drv8434s_t *dev, uint8_t flags)
{
    if (!dev)
        return false;

    // CTRL4 bits [3:0] map directly from flags bits [3:0]:
    //   bit 0 = OTW_REP, bit 1 = OTSD_MODE, bit 2 = OCP_MODE, bit 3 = EN_OL
    uint8_t ctrl4_mask = DRV8434S_CTRL4_OTW_REP | DRV8434S_CTRL4_OTSD_MODE |
                         DRV8434S_CTRL4_OCP_MODE | DRV8434S_CTRL4_EN_OL;
    uint8_t ctrl4_val = flags & ctrl4_mask;

    if (!drv8434s_modify_reg(dev, DRV8434S_REG_CTRL4, ctrl4_mask, ctrl4_val))
        return false;

    // CTRL5 STL_REP (bit 3) ← flags bit 4 (DRV8434S_FAULT_CFG_STL_REP)
    uint8_t stl_rep = (flags & DRV8434S_FAULT_CFG_STL_REP)
                          ? DRV8434S_CTRL5_STL_REP
                          : 0;
    return drv8434s_modify_reg(dev, DRV8434S_REG_CTRL5,
                               DRV8434S_CTRL5_STL_REP, stl_rep);
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

// ═════════════════════════════════════════════════════════════════════════════
//  Daisy Chain API  (Section 7.5.1.3)
// ═════════════════════════════════════════════════════════════════════════════
//
// TX frame layout for N devices (indices are 0-based, byte 0 first on wire):
//   [HDR1][HDR2][A_{N-1}]...[A_0][D_{N-1}]...[D_0]
//
//   Device k (0 = closest to MCU MOSI) occupies:
//     TX addr : tx[ 2 + (N-1-k) ]
//     TX data : tx[ 2 + N + (N-1-k) ]
//
// RX frame layout (received simultaneously from the last device's SDO):
//   [S_{N-1}]...[S_0][HDR1][HDR2][R_{N-1}]...[R_0]
//
//     RX status : rx[ N-1-k ]
//     RX report : rx[ 2 + N + (N-1-k) ]   ← same offset as TX data
//
// Verification for N=3 (devices 0,1,2):
//   TX : [HDR1,HDR2, A2,A1,A0, D2,D1,D0]
//          0    1    2  3  4   5  6  7
//   k=0: addr@4, data@7   k=1: addr@3, data@6   k=2: addr@2, data@5  ✓
//   RX : [S2, S1, S0, HDR1,HDR2, R2,R1,R0]
//          0   1   2   3    4    5  6  7
//   k=0: stat@2, rprt@7   k=1: stat@1, rprt@6   k=2: stat@0, rprt@5  ✓

#define CHAIN_TX_ADDR(N, k) (2u + ((N) - 1u - (k)))
#define CHAIN_TX_DATA(N, k) (2u + (N) + ((N) - 1u - (k)))
#define CHAIN_RX_STAT(N, k) ((N) - 1u - (k))
#define CHAIN_RX_RPRT(N, k) (2u + (N) + ((N) - 1u - (k)))

// NOP command: read FAULT register (address 0x00) — no state change on the device.
#define CHAIN_NOP_ADDR \
    (DRV8434S_SPI_READ_BIT | ((DRV8434S_REG_FAULT & DRV8434S_ADDR_MASK) << DRV8434S_ADDR_SHIFT))
#define CHAIN_NOP_DATA 0x00u

// ── Internal helpers ──────────────────────────────────────────────────────────

static inline void drv_delay_us_chain(drv8434s_chain_t *chain, unsigned us)
{
    if (chain->cfg.delay_us)
    {
        chain->cfg.delay_us(chain->cfg.user_ctx, us);
    }
    else if (chain->cfg.delay_ms && us >= 1000)
    {
        chain->cfg.delay_ms(chain->cfg.user_ctx, us / 1000);
    }
}

static inline uint8_t chain_frame_len(const drv8434s_chain_t *c)
{
    return (uint8_t)(2u + 2u * c->cfg.n_devices);
}

// Assert CS, transfer one full chain frame, deassert CS, apply hold delay.
static int chain_xfer(drv8434s_chain_t *chain, const uint8_t *tx, uint8_t *rx)
{
    uint8_t len = chain_frame_len(chain);
    chain->cfg.cs(chain->cfg.user_ctx, true);
    int r = chain->cfg.spi_xfer(chain->cfg.user_ctx, tx, rx, len);
    chain->cfg.cs(chain->cfg.user_ctx, false);
    if (chain->cfg.delay_us)
        chain->cfg.delay_us(chain->cfg.user_ctx, DRV8434S_CS_HOLD_US);
    return r;
}

// Fill a TX buffer from per-device address and data byte arrays.
// addr_bytes[k] / data_bytes[k] correspond to device k (0 = closest to MCU).
static void chain_fill_tx(const drv8434s_chain_t *chain,
                          const uint8_t *addr_bytes,
                          const uint8_t *data_bytes,
                          uint8_t *tx)
{
    uint8_t N = chain->cfg.n_devices;
    tx[0] = DRV8434S_CHAIN_HDR_PREFIX | (N & DRV8434S_CHAIN_HDR_N_MASK);
    tx[1] = DRV8434S_CHAIN_HDR_PREFIX; // lower bits don't care
    for (uint8_t k = 0; k < N; ++k)
    {
        tx[CHAIN_TX_ADDR(N, k)] = addr_bytes[k];
        tx[CHAIN_TX_DATA(N, k)] = data_bytes[k];
    }
}

// Parse an RX frame into optional per-device status and report arrays.
// status_out[k]: status byte for device k; report_out[k]: report/read byte.
static void chain_parse_rx(const drv8434s_chain_t *chain,
                           const uint8_t *rx,
                           drv8434s_chain_dev_status_t *status_out,
                           uint8_t *report_out)
{
    uint8_t N = chain->cfg.n_devices;
    for (uint8_t k = 0; k < N; ++k)
    {
        if (status_out)
        {
            uint8_t s = rx[CHAIN_RX_STAT(N, k)];
            status_out[k].raw = s;
            // Bits[5:0] carry fault flags; non-zero means a fault is active.
            status_out[k].is_faulted = (s & DRV8434S_CHAIN_STAT_MASK) != 0u;
        }
        if (report_out)
            report_out[k] = rx[CHAIN_RX_RPRT(N, k)];
    }
}

// ── Public API ────────────────────────────────────────────────────────────────

bool drv8434s_chain_init(drv8434s_chain_t *chain,
                         const drv8434s_chain_config_t *cfg)
{
    if (!chain || !cfg || !cfg->spi_xfer || !cfg->cs)
        return false;
    if (cfg->n_devices == 0u || cfg->n_devices > DRV8434S_CHAIN_MAX_DEVICES)
        return false;

    memset(chain, 0, sizeof(*chain));
    chain->cfg = *cfg;
    return true;
}

bool drv8434s_chain_write_reg(drv8434s_chain_t *chain,
                              uint8_t dev_idx, uint8_t reg, uint8_t value,
                              drv8434s_chain_dev_status_t *status_out)
{
    if (!chain || dev_idx >= chain->cfg.n_devices)
        return false;

    uint8_t N = chain->cfg.n_devices;
    uint8_t tx[DRV8434S_CHAIN_MAX_FRAME] = {0};
    uint8_t rx[DRV8434S_CHAIN_MAX_FRAME] = {0};
    uint8_t addr_bytes[DRV8434S_CHAIN_MAX_DEVICES];
    uint8_t data_bytes[DRV8434S_CHAIN_MAX_DEVICES];

    // All devices default to NOP; override the target device.
    for (uint8_t k = 0; k < N; ++k)
    {
        addr_bytes[k] = CHAIN_NOP_ADDR;
        data_bytes[k] = CHAIN_NOP_DATA;
    }
    addr_bytes[dev_idx] = DRV8434S_SPI_WRITE |
                          ((reg & DRV8434S_ADDR_MASK) << DRV8434S_ADDR_SHIFT);
    data_bytes[dev_idx] = value;

    chain_fill_tx(chain, addr_bytes, data_bytes, tx);
    if (chain_xfer(chain, tx, rx) != 0)
        return false;

    // Post-write settling before the next SPI frame.
    if (chain->cfg.delay_us)
        chain->cfg.delay_us(chain->cfg.user_ctx, DRV8434S_REG_SETTLE_US);

    // Update shadow cache for the target device.
    if ((reg & DRV8434S_ADDR_MASK) < DRV8434S_NUM_REGS)
        chain->reg_cache[dev_idx][reg & DRV8434S_ADDR_MASK] = value;

    chain_parse_rx(chain, rx, status_out, NULL);
    return true;
}

bool drv8434s_chain_read_reg(drv8434s_chain_t *chain,
                             uint8_t dev_idx, uint8_t reg, uint8_t *value,
                             drv8434s_chain_dev_status_t *status_out)
{
    if (!chain || !value || dev_idx >= chain->cfg.n_devices)
        return false;

    uint8_t N = chain->cfg.n_devices;
    uint8_t tx[DRV8434S_CHAIN_MAX_FRAME] = {0};
    uint8_t rx[DRV8434S_CHAIN_MAX_FRAME];
    memset(rx, 0xFF, sizeof(rx));
    uint8_t addr_bytes[DRV8434S_CHAIN_MAX_DEVICES];
    uint8_t data_bytes[DRV8434S_CHAIN_MAX_DEVICES];

    // All devices default to NOP; override the target device with the read.
    for (uint8_t k = 0; k < N; ++k)
    {
        addr_bytes[k] = CHAIN_NOP_ADDR;
        data_bytes[k] = CHAIN_NOP_DATA;
    }
    addr_bytes[dev_idx] = DRV8434S_SPI_READ_BIT |
                          ((reg & DRV8434S_ADDR_MASK) << DRV8434S_ADDR_SHIFT);

    chain_fill_tx(chain, addr_bytes, data_bytes, tx);
    if (chain_xfer(chain, tx, rx) != 0)
        return false;

    uint8_t report[DRV8434S_CHAIN_MAX_DEVICES];
    chain_parse_rx(chain, rx, status_out, report);

    *value = report[dev_idx];
    if ((reg & DRV8434S_ADDR_MASK) < DRV8434S_NUM_REGS)
        chain->reg_cache[dev_idx][reg & DRV8434S_ADDR_MASK] = *value;

    return true;
}

bool drv8434s_chain_modify_reg(drv8434s_chain_t *chain,
                               uint8_t dev_idx, uint8_t reg,
                               uint8_t mask, uint8_t value,
                               drv8434s_chain_dev_status_t *status_out)
{
    uint8_t current;
    if (!drv8434s_chain_read_reg(chain, dev_idx, reg, &current, status_out))
        return false;
    uint8_t updated = (current & ~mask) | (value & mask);
    return drv8434s_chain_write_reg(chain, dev_idx, reg, updated, status_out);
}

bool drv8434s_chain_write_all(drv8434s_chain_t *chain,
                              uint8_t reg, const uint8_t *values,
                              drv8434s_chain_dev_status_t *status_out)
{
    if (!chain || !values)
        return false;

    uint8_t N = chain->cfg.n_devices;
    uint8_t tx[DRV8434S_CHAIN_MAX_FRAME] = {0};
    uint8_t rx[DRV8434S_CHAIN_MAX_FRAME] = {0};
    uint8_t addr_bytes[DRV8434S_CHAIN_MAX_DEVICES];
    uint8_t data_bytes[DRV8434S_CHAIN_MAX_DEVICES];

    for (uint8_t k = 0; k < N; ++k)
    {
        addr_bytes[k] = DRV8434S_SPI_WRITE |
                        ((reg & DRV8434S_ADDR_MASK) << DRV8434S_ADDR_SHIFT);
        data_bytes[k] = values[k];
    }

    chain_fill_tx(chain, addr_bytes, data_bytes, tx);
    if (chain_xfer(chain, tx, rx) != 0)
        return false;

    if (chain->cfg.delay_us)
        chain->cfg.delay_us(chain->cfg.user_ctx, DRV8434S_REG_SETTLE_US);

    if ((reg & DRV8434S_ADDR_MASK) < DRV8434S_NUM_REGS)
        for (uint8_t k = 0; k < N; ++k)
            chain->reg_cache[k][reg & DRV8434S_ADDR_MASK] = values[k];

    chain_parse_rx(chain, rx, status_out, NULL);
    return true;
}

bool drv8434s_chain_read_all(drv8434s_chain_t *chain,
                             uint8_t reg, uint8_t *values_out,
                             drv8434s_chain_dev_status_t *status_out)
{
    if (!chain || !values_out)
        return false;

    uint8_t N = chain->cfg.n_devices;
    uint8_t tx[DRV8434S_CHAIN_MAX_FRAME] = {0};
    uint8_t rx[DRV8434S_CHAIN_MAX_FRAME];
    memset(rx, 0xFF, sizeof(rx));
    uint8_t addr_bytes[DRV8434S_CHAIN_MAX_DEVICES];
    uint8_t data_bytes[DRV8434S_CHAIN_MAX_DEVICES];

    for (uint8_t k = 0; k < N; ++k)
    {
        addr_bytes[k] = DRV8434S_SPI_READ_BIT |
                        ((reg & DRV8434S_ADDR_MASK) << DRV8434S_ADDR_SHIFT);
        data_bytes[k] = 0x00u;
    }

    chain_fill_tx(chain, addr_bytes, data_bytes, tx);
    if (chain_xfer(chain, tx, rx) != 0)
        return false;

    chain_parse_rx(chain, rx, status_out, values_out);

    if ((reg & DRV8434S_ADDR_MASK) < DRV8434S_NUM_REGS)
        for (uint8_t k = 0; k < N; ++k)
            chain->reg_cache[k][reg & DRV8434S_ADDR_MASK] = values_out[k];

    return true;
}

// ═════════════════════════════════════════════════════════════════════════════
//  Per-device chain motor-control helpers
// ═════════════════════════════════════════════════════════════════════════════

bool drv8434s_chain_enable(drv8434s_chain_t *chain, uint8_t dev_idx,
                           drv8434s_chain_dev_status_t *status_out)
{
    if (!chain || dev_idx >= chain->cfg.n_devices)
        return false;
    uint8_t ridx = DRV8434S_REG_CTRL2 & DRV8434S_ADDR_MASK;
    uint8_t val = chain->reg_cache[dev_idx][ridx] | DRV8434S_CTRL2_EN_OUT;
    return drv8434s_chain_write_reg(chain, dev_idx, DRV8434S_REG_CTRL2,
                                    val, status_out);
}

bool drv8434s_chain_disable(drv8434s_chain_t *chain, uint8_t dev_idx,
                            drv8434s_chain_dev_status_t *status_out)
{
    if (!chain || dev_idx >= chain->cfg.n_devices)
        return false;
    uint8_t ridx = DRV8434S_REG_CTRL2 & DRV8434S_ADDR_MASK;
    uint8_t val = chain->reg_cache[dev_idx][ridx] & (uint8_t)~DRV8434S_CTRL2_EN_OUT;
    return drv8434s_chain_write_reg(chain, dev_idx, DRV8434S_REG_CTRL2,
                                    val, status_out);
}

bool drv8434s_chain_step(drv8434s_chain_t *chain, uint8_t dev_idx,
                         drv8434s_chain_dev_status_t *status_out)
{
    if (!chain || dev_idx >= chain->cfg.n_devices)
        return false;

    uint8_t ridx = DRV8434S_REG_CTRL3 & DRV8434S_ADDR_MASK;

    // Strip STEP from the cached base so the device sees a clean 0→1 edge.
    uint8_t base = chain->reg_cache[dev_idx][ridx] & (uint8_t)~DRV8434S_CTRL3_STEP;
    uint8_t val = base | DRV8434S_CTRL3_STEP;

    if (!drv8434s_chain_write_reg(chain, dev_idx, DRV8434S_REG_CTRL3,
                                  val, status_out))
        return false;

    // STEP is self-clearing — restore the cache to STEP=0 so the next call
    // always produces a rising edge.
    chain->reg_cache[dev_idx][ridx] = base;
    return true;
}

bool drv8434s_chain_set_dir(drv8434s_chain_t *chain, uint8_t dev_idx,
                            bool reverse,
                            drv8434s_chain_dev_status_t *status_out)
{
    if (!chain || dev_idx >= chain->cfg.n_devices)
        return false;
    uint8_t ridx = DRV8434S_REG_CTRL3 & DRV8434S_ADDR_MASK;
    uint8_t base = chain->reg_cache[dev_idx][ridx];
    uint8_t val = reverse ? (base | DRV8434S_CTRL3_DIR)
                          : (base & (uint8_t)~DRV8434S_CTRL3_DIR);
    return drv8434s_chain_write_reg(chain, dev_idx, DRV8434S_REG_CTRL3,
                                    val, status_out);
}

bool drv8434s_chain_set_microstep(drv8434s_chain_t *chain, uint8_t dev_idx,
                                  drv8434s_microstep_t mode,
                                  drv8434s_chain_dev_status_t *status_out)
{
    if (!chain || dev_idx >= chain->cfg.n_devices || (uint8_t)mode > 9u)
        return false;
    return drv8434s_chain_modify_reg(chain, dev_idx, DRV8434S_REG_CTRL3,
                                     DRV8434S_CTRL3_MICROSTEP_MASK,
                                     (uint8_t)mode << DRV8434S_CTRL3_MICROSTEP_SHIFT,
                                     status_out);
}

bool drv8434s_chain_set_torque(drv8434s_chain_t *chain, uint8_t dev_idx,
                               uint8_t trq,
                               drv8434s_chain_dev_status_t *status_out)
{
    if (!chain || dev_idx >= chain->cfg.n_devices)
        return false;
    if (trq > 15)
        trq = 15;
    return drv8434s_chain_modify_reg(chain, dev_idx, DRV8434S_REG_CTRL1,
                                     DRV8434S_CTRL1_TRQ_DAC_MASK,
                                     trq << DRV8434S_CTRL1_TRQ_DAC_SHIFT,
                                     status_out);
}

bool drv8434s_chain_set_spi_step_mode(drv8434s_chain_t *chain,
                                      uint8_t dev_idx,
                                      drv8434s_chain_dev_status_t *status_out)
{
    if (!chain || dev_idx >= chain->cfg.n_devices)
        return false;
    return drv8434s_chain_modify_reg(chain, dev_idx, DRV8434S_REG_CTRL3,
                                     DRV8434S_CTRL3_SPI_STEP | DRV8434S_CTRL3_SPI_DIR,
                                     DRV8434S_CTRL3_SPI_STEP | DRV8434S_CTRL3_SPI_DIR,
                                     status_out);
}

bool drv8434s_chain_read_torque_count(drv8434s_chain_t *chain,
                                      uint8_t dev_idx,
                                      uint16_t *torque_out,
                                      drv8434s_chain_dev_status_t *status_out)
{
    if (!chain || !torque_out || dev_idx >= chain->cfg.n_devices)
        return false;

    // TRQ_COUNT low 8 bits in CTRL8 (0x0A)
    uint8_t low = 0;
    if (!drv8434s_chain_read_reg(chain, dev_idx, DRV8434S_REG_CTRL8,
                                 &low, status_out))
        return false;

    // TRQ_COUNT high 4 bits in CTRL9 (0x0B) bits [3:0]
    uint8_t high = 0;
    if (!drv8434s_chain_read_reg(chain, dev_idx, DRV8434S_REG_CTRL9,
                                 &high, status_out))
        return false;

    *torque_out = ((uint16_t)(high & DRV8434S_CTRL9_TRQ_COUNT_HI_MASK) << 8) | low;
    return true;
}

bool drv8434s_chain_clear_faults(drv8434s_chain_t *chain, uint8_t dev_idx,
                                 drv8434s_chain_dev_status_t *status_out)
{
    if (!chain || dev_idx >= chain->cfg.n_devices)
        return false;
    return drv8434s_chain_modify_reg(chain, dev_idx, DRV8434S_REG_CTRL4,
                                     DRV8434S_CTRL4_CLR_FLT,
                                     DRV8434S_CTRL4_CLR_FLT,
                                     status_out);
}

bool drv8434s_chain_set_fault_config(drv8434s_chain_t *chain,
                                     uint8_t dev_idx, uint8_t flags,
                                     drv8434s_chain_dev_status_t *status_out)
{
    if (!chain || dev_idx >= chain->cfg.n_devices)
        return false;

    // CTRL4 bits [3:0]: OTW_REP, OTSD_MODE, OCP_MODE, EN_OL
    uint8_t ctrl4_mask = DRV8434S_CTRL4_OTW_REP | DRV8434S_CTRL4_OTSD_MODE |
                         DRV8434S_CTRL4_OCP_MODE | DRV8434S_CTRL4_EN_OL;
    uint8_t ctrl4_val = flags & ctrl4_mask;

    if (!drv8434s_chain_modify_reg(chain, dev_idx, DRV8434S_REG_CTRL4,
                                   ctrl4_mask, ctrl4_val, status_out))
        return false;

    // CTRL5 STL_REP (bit 3) ← flags bit 4 (DRV8434S_FAULT_CFG_STL_REP)
    uint8_t stl_rep = (flags & DRV8434S_FAULT_CFG_STL_REP)
                          ? DRV8434S_CTRL5_STL_REP
                          : 0;
    return drv8434s_chain_modify_reg(chain, dev_idx, DRV8434S_REG_CTRL5,
                                     DRV8434S_CTRL5_STL_REP, stl_rep,
                                     status_out);
}

// ═════════════════════════════════════════════════════════════════════════════
//  Non-blocking motion engine
// ═════════════════════════════════════════════════════════════════════════════

// Helper: build a result snapshot from a job's current state.
static void motion_build_result(const drv8434s_motion_job_t *job,
                                drv8434s_motion_result_t *result)
{
    result->steps_requested = job->steps_requested;
    result->steps_achieved = job->steps_achieved;
    result->last_torque_count = job->last_torque_count;
    result->reason = job->reason;
}

// Helper: finish a job, invoke done_cb, and check whether the engine can idle.
static void motion_finish_job(drv8434s_motion_t *motion, uint8_t dev_idx)
{
    drv8434s_motion_job_t *job = &motion->jobs[dev_idx];
    job->active = false;

    if (motion->done_cb)
    {
        drv8434s_motion_result_t result;
        motion_build_result(job, &result);
        motion->done_cb(motion->done_ctx, dev_idx, &result);
    }
}

bool drv8434s_motion_init(drv8434s_motion_t *motion,
                          drv8434s_chain_t *chain,
                          drv8434s_motion_done_cb_t done_cb,
                          void *done_ctx,
                          uint8_t torque_sample_div)
{
    if (!motion || !chain)
        return false;

    memset(motion, 0, sizeof(*motion));
    motion->chain = chain;
    motion->done_cb = done_cb;
    motion->done_ctx = done_ctx;
    motion->torque_sample_div = (torque_sample_div == 0) ? 1 : torque_sample_div;
    return true;
}

bool drv8434s_motion_start(drv8434s_motion_t *motion, uint8_t dev_idx,
                           int32_t target_steps, uint16_t torque_limit)
{
    if (!motion || !motion->chain || dev_idx >= motion->chain->cfg.n_devices)
        return false;

    drv8434s_motion_job_t *job = &motion->jobs[dev_idx];
    if (job->active)
        return false; // Already running on this device

    if (target_steps == 0)
        return true; // Nothing to do

    memset(job, 0, sizeof(*job));
    job->active = true;
    job->reverse = (target_steps < 0);
    job->steps_requested = target_steps;
    job->steps_remaining = job->reverse ? -target_steps : target_steps;
    job->torque_limit = torque_limit;
    job->reason = DRV8434S_MOTION_OK;

    // Set direction on the device
    if (!drv8434s_chain_set_dir(motion->chain, dev_idx, job->reverse, NULL))
    {
        job->active = false;
        job->reason = DRV8434S_MOTION_SPI_ERROR;
        return false;
    }

    return true;
}

bool drv8434s_motion_tick(drv8434s_motion_t *motion)
{
    if (!motion || !motion->chain)
        return false;

    drv8434s_chain_t *chain = motion->chain;
    uint8_t N = chain->cfg.n_devices;
    uint8_t ridx = DRV8434S_REG_CTRL3 & DRV8434S_ADDR_MASK;

    // Count active jobs — if none, nothing to do.
    bool any_active = false;
    for (uint8_t k = 0; k < N; ++k)
    {
        if (motion->jobs[k].active)
        {
            any_active = true;
            break;
        }
    }
    if (!any_active)
        return true;

    // ── Step all active motors in a single chain frame ──────────────────
    uint8_t tx[DRV8434S_CHAIN_MAX_FRAME] = {0};
    uint8_t rx[DRV8434S_CHAIN_MAX_FRAME] = {0};
    uint8_t addr_bytes[DRV8434S_CHAIN_MAX_DEVICES];
    uint8_t data_bytes[DRV8434S_CHAIN_MAX_DEVICES];

    uint8_t addr_val = DRV8434S_SPI_WRITE |
                       ((DRV8434S_REG_CTRL3 & DRV8434S_ADDR_MASK) << DRV8434S_ADDR_SHIFT);

    for (uint8_t k = 0; k < N; ++k)
    {
        if (motion->jobs[k].active)
        {
            uint8_t base = chain->reg_cache[k][ridx] & (uint8_t)~DRV8434S_CTRL3_STEP;
            addr_bytes[k] = addr_val;
            data_bytes[k] = base | DRV8434S_CTRL3_STEP;
        }
        else
        {
            addr_bytes[k] = CHAIN_NOP_ADDR;
            data_bytes[k] = CHAIN_NOP_DATA;
        }
    }

    chain_fill_tx(chain, addr_bytes, data_bytes, tx);
    if (chain_xfer(chain, tx, rx) != 0)
    {
        // SPI failure: abort all active jobs
        for (uint8_t k = 0; k < N; ++k)
        {
            if (motion->jobs[k].active)
            {
                motion->jobs[k].reason = DRV8434S_MOTION_SPI_ERROR;
                motion_finish_job(motion, k);
            }
        }
        return false;
    }

    // Restore STEP=0 in cache for active devices (STEP is self-clearing)
    for (uint8_t k = 0; k < N; ++k)
    {
        if (motion->jobs[k].active)
            chain->reg_cache[k][ridx] &= (uint8_t)~DRV8434S_CTRL3_STEP;
    }

    // Parse status bytes for fault checking
    drv8434s_chain_dev_status_t status[DRV8434S_CHAIN_MAX_DEVICES];
    chain_parse_rx(chain, rx, status, NULL);

    // ── Update each active job ──────────────────────────────────────────
    // Collect indices to finish after the loop to avoid modifying while iterating
    uint8_t finish_list[DRV8434S_CHAIN_MAX_DEVICES];
    uint8_t finish_count = 0;

    for (uint8_t k = 0; k < N; ++k)
    {
        drv8434s_motion_job_t *job = &motion->jobs[k];
        if (!job->active)
            continue;

        job->steps_achieved += job->reverse ? -1 : 1;
        job->steps_remaining--;

        if (status[k].is_faulted)
        {
            job->reason = DRV8434S_MOTION_FAULT;
            finish_list[finish_count++] = k;
        }
        else if (job->steps_remaining <= 0)
        {
            job->reason = DRV8434S_MOTION_OK;
            finish_list[finish_count++] = k;
        }
    }

    // ── Torque sampling (periodic) ──────────────────────────────────────
    motion->torque_tick_count++;
    bool do_torque = (motion->torque_tick_count >= motion->torque_sample_div);
    if (do_torque)
    {
        motion->torque_tick_count = 0;

        for (uint8_t k = 0; k < N; ++k)
        {
            drv8434s_motion_job_t *job = &motion->jobs[k];
            if (!job->active || job->torque_limit == 0)
                continue;

            // Skip if already scheduled to finish
            bool already_done = false;
            for (uint8_t f = 0; f < finish_count; ++f)
            {
                if (finish_list[f] == k)
                {
                    already_done = true;
                    break;
                }
            }
            if (already_done)
                continue;

            uint16_t trq = 0;
            if (!drv8434s_chain_read_torque_count(chain, k, &trq, NULL))
            {
                job->reason = DRV8434S_MOTION_SPI_ERROR;
                finish_list[finish_count++] = k;
                continue;
            }
            job->last_torque_count = trq;

            // Push into rolling-average buffer
            job->trq_buf[job->trq_buf_idx] = trq;
            job->trq_buf_idx = (job->trq_buf_idx + 1u) % DRV8434S_MOTION_TRQ_BUF_SIZE;
            if (job->trq_buf_count < DRV8434S_MOTION_TRQ_BUF_SIZE)
                job->trq_buf_count++;

            // Compute average
            uint32_t sum = 0;
            for (uint8_t b = 0; b < job->trq_buf_count; ++b)
                sum += job->trq_buf[b];
            uint16_t avg = (uint16_t)(sum / job->trq_buf_count);
// Tuning print
#ifdef TUNING
            printf("Motor %u with torque %u: [%u,%u,%u,%u,%u,%u,%u,%u,%u,%u]\n", k, avg, job->trq_buf[0], job->trq_buf[1], job->trq_buf[2], job->trq_buf[3], job->trq_buf[4], job->trq_buf[5], job->trq_buf[6], job->trq_buf[7], job->trq_buf[8], job->trq_buf[9]);
#endif

            // TRQ_COUNT = torque margin until stall (0 = stalled).
            // Trip when average falls at or below the limit.
            if (avg <= job->torque_limit)
            {
                job->reason = DRV8434S_MOTION_TORQUE_LIMIT;
                finish_list[finish_count++] = k;
            }
        }
    }

    // ── Fire completion callbacks ────────────────────────────────────────
    for (uint8_t f = 0; f < finish_count; ++f)
        motion_finish_job(motion, finish_list[f]);

    return true;
}

bool drv8434s_motion_cancel(drv8434s_motion_t *motion, uint8_t dev_idx,
                            drv8434s_motion_result_t *result)
{
    if (!motion || !motion->chain || dev_idx >= motion->chain->cfg.n_devices)
        return false;

    drv8434s_motion_job_t *job = &motion->jobs[dev_idx];
    if (!job->active)
        return false;

    job->reason = DRV8434S_MOTION_CANCELLED;
    job->active = false;

    if (result)
        motion_build_result(job, result);

    // done_cb is NOT invoked for cancellations (caller already knows).
    return true;
}

bool drv8434s_motion_is_busy(const drv8434s_motion_t *motion)
{
    if (!motion || !motion->chain)
        return false;

    for (uint8_t k = 0; k < motion->chain->cfg.n_devices; ++k)
    {
        if (motion->jobs[k].active)
            return true;
    }
    return false;
}