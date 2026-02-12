#ifndef DRV8434S_H
#define DRV8434S_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C"
{
#endif

    // ── Portable driver API for DRV8434S SPI stepper motor driver ───────────
    //
    // The API is hardware-agnostic: callers supply small callback functions
    // for SPI transfers, GPIO control, and delay.  The lower layer (init,
    // probe, reset) is unchanged from the minimal driver; the upper layer
    // adds register-level control, SPI stepping, and torque monitoring.
    //
    // Reference: TI DRV8434S datasheet – Table 7-15 Memory Map
    //   SPI Mode 1 (CPOL = 0, CPHA = 1), 16-bit frames, MSB-first
    //   Frame (16 bits, MSB first):
    //     Bit 15      = don't care (set to 0)
    //     Bit 14      = W  (1 = read, 0 = write)
    //     Bits 13:9   = A4:A0  (5-bit register address)
    //     Bit 8       = don't care (set to 0)
    //     Bits 7:0    = D7:D0  (8-bit data)
    //
    //   Byte-level view (MSB-first):
    //     Byte 0: [0][W][A4][A3][A2][A1][A0][0]
    //     Byte 1: [D7][D6][D5][D4][D3][D2][D1][D0]

    // ── SPI protocol constants ──────────────────────────────────────────────

#define DRV8434S_SPI_READ_BIT 0x40 // Byte 0 bit 6 = frame bit 14 → read
#define DRV8434S_SPI_WRITE 0x00    // Byte 0 bit 6 = 0 → write
#define DRV8434S_ADDR_MASK 0x1F    // 5-bit address before shift (A4:A0)
#define DRV8434S_ADDR_SHIFT 1      // Address sits in byte 0 bits [5:1]

    // ── Register addresses (Table 7-15) ────────────────────────────────────

#define DRV8434S_REG_FAULT 0x00 // Fault status            (R,  reset 0x00)
#define DRV8434S_REG_DIAG1 0x01 // OCP diagnostic          (R,  reset 0x00)
#define DRV8434S_REG_DIAG2 0x02 // OTW / OTS / stall diag  (R,  reset 0x00)
#define DRV8434S_REG_CTRL1 0x03 // TRQ_DAC, OL_MODE        (RW, reset 0x00)
#define DRV8434S_REG_CTRL2 0x04 // EN_OUT, TOFF, DECAY     (RW, reset 0x0F)
#define DRV8434S_REG_CTRL3 0x05 // Step/Dir, microstep     (RW, reset 0x06)
#define DRV8434S_REG_CTRL4 0x06 // CLR_FLT, LOCK, EN_OL    (RW, reset 0x30)
#define DRV8434S_REG_CTRL5 0x07 // Stall learning/enable   (RW, reset 0x08)
#define DRV8434S_REG_CTRL6 0x08 // STALL_TH [7:0]          (RW, reset 0x03)
#define DRV8434S_REG_CTRL7 0x09 // RC_RIPPLE, SSC, scale   (RW, reset 0x20)
#define DRV8434S_REG_CTRL8 0x0A // TRQ_COUNT [7:0]         (R,  reset 0x00)
#define DRV8434S_REG_CTRL9 0x0B // REV_ID, TRQ_COUNT[11:8] (R,  reset 0x00)

#define DRV8434S_NUM_REGS 12 // Total registers (0x00 – 0x0B)

    // ── FAULT register (0x00) bit masks ─────────────────────────────────────

#define DRV8434S_FAULT_FAULT (1u << 7)   // Any fault active
#define DRV8434S_FAULT_SPI_ERR (1u << 6) // SPI protocol error
#define DRV8434S_FAULT_UVLO (1u << 5)    // Supply under-voltage
#define DRV8434S_FAULT_CPUV (1u << 4)    // Charge-pump under-voltage
#define DRV8434S_FAULT_OCP (1u << 3)     // Over-current
#define DRV8434S_FAULT_STL (1u << 2)     // Stall detected
#define DRV8434S_FAULT_TF (1u << 1)      // Thermal flag
#define DRV8434S_FAULT_OL (1u << 0)      // Open load

    // ── DIAG1 register (0x01) bit masks ─────────────────────────────────────

#define DRV8434S_DIAG1_OCP_LS2_B (1u << 7)
#define DRV8434S_DIAG1_OCP_HS2_B (1u << 6)
#define DRV8434S_DIAG1_OCP_LS1_B (1u << 5)
#define DRV8434S_DIAG1_OCP_HS1_B (1u << 4)
#define DRV8434S_DIAG1_OCP_LS2_A (1u << 3)
#define DRV8434S_DIAG1_OCP_HS2_A (1u << 2)
#define DRV8434S_DIAG1_OCP_LS1_A (1u << 1)
#define DRV8434S_DIAG1_OCP_HS1_A (1u << 0)

    // ── DIAG2 register (0x02) bit masks ─────────────────────────────────────

#define DRV8434S_DIAG2_OTW (1u << 6)        // Over-temperature warning
#define DRV8434S_DIAG2_OTS (1u << 5)        // Over-temperature shutdown
#define DRV8434S_DIAG2_STL_LRN_OK (1u << 4) // Stall learning complete
#define DRV8434S_DIAG2_STALL (1u << 3)      // Stall detected
#define DRV8434S_DIAG2_OL_B (1u << 1)       // Open load coil B
#define DRV8434S_DIAG2_OL_A (1u << 0)       // Open load coil A

    // ── CTRL1 register (0x03) bit masks ─────────────────────────────────────

#define DRV8434S_CTRL1_TRQ_DAC_MASK 0x78 // Bits [6:3]
#define DRV8434S_CTRL1_TRQ_DAC_SHIFT 3
#define DRV8434S_CTRL1_OL_MODE (1u << 1) // Open-load mode select

    // ── CTRL2 register (0x04) bit masks ─────────────────────────────────────

#define DRV8434S_CTRL2_EN_OUT (1u << 7) // 1 = outputs enabled
#define DRV8434S_CTRL2_TOFF_MASK 0x30   // Bits [5:4] PWM off-time
#define DRV8434S_CTRL2_TOFF_SHIFT 4
#define DRV8434S_CTRL2_DECAY_MASK 0x07 // Bits [2:0] decay mode
#define DRV8434S_CTRL2_DECAY_SHIFT 0

    // ── CTRL3 register (0x05) bit masks ─────────────────────────────────────

#define DRV8434S_CTRL3_DIR (1u << 7)       // Direction control (when SPI_DIR=1)
#define DRV8434S_CTRL3_STEP (1u << 6)      // Step trigger (when SPI_STEP=1, rising edge)
#define DRV8434S_CTRL3_SPI_DIR (1u << 5)   // 1 = SPI controls DIR, 0 = HW pin
#define DRV8434S_CTRL3_SPI_STEP (1u << 4)  // 1 = SPI controls STEP, 0 = HW pin
#define DRV8434S_CTRL3_MICROSTEP_MASK 0x0F // Bits [3:0]
#define DRV8434S_CTRL3_MICROSTEP_SHIFT 0

    // ── CTRL4 register (0x06) bit masks ─────────────────────────────────────

#define DRV8434S_CTRL4_CLR_FLT (1u << 7) // Write 1 to clear faults
#define DRV8434S_CTRL4_LOCK_MASK 0x70    // Bits [6:4] lock registers
#define DRV8434S_CTRL4_LOCK_SHIFT 4
#define DRV8434S_CTRL4_EN_OL (1u << 3)     // Open-load detection enable
#define DRV8434S_CTRL4_OCP_MODE (1u << 2)  // Over-current mode select
#define DRV8434S_CTRL4_OTSD_MODE (1u << 1) // Over-temp shutdown mode
#define DRV8434S_CTRL4_OTW_REP (1u << 0)   // Over-temp warning report

    // ── CTRL5 register (0x07) bit masks ─────────────────────────────────────

#define DRV8434S_CTRL5_STL_LRN (1u << 5) // Start stall learning
#define DRV8434S_CTRL5_EN_STL (1u << 4)  // Enable stall detection
#define DRV8434S_CTRL5_STL_REP (1u << 3) // Stall report on nFAULT

    // ── CTRL6 register (0x08) bit masks ─────────────────────────────────────

#define DRV8434S_CTRL6_STALL_TH_MASK 0xFF // Bits [7:0] stall threshold low

    // ── CTRL7 register (0x09) bit masks ─────────────────────────────────────

#define DRV8434S_CTRL7_RC_RIPPLE_MASK 0xC0 // Bits [7:6] ripple control
#define DRV8434S_CTRL7_RC_RIPPLE_SHIFT 6
#define DRV8434S_CTRL7_EN_SSC (1u << 5)      // Spread-spectrum clocking
#define DRV8434S_CTRL7_TRQ_SCALE (1u << 4)   // Torque count scale
#define DRV8434S_CTRL7_STALL_TH_HI_MASK 0x0F // Bits [3:0] STALL_TH[11:8]

    // ── CTRL8 / TRQ_COUNT register (0x0A) ──────────────────────────────────

#define DRV8434S_CTRL8_TRQ_COUNT_MASK 0xFF // Bits [7:0] torque count low

    // ── CTRL9 register (0x0B) ──────────────────────────────────────────────

#define DRV8434S_CTRL9_REV_ID_MASK 0xF0 // Bits [7:4] revision ID
#define DRV8434S_CTRL9_REV_ID_SHIFT 4
#define DRV8434S_CTRL9_TRQ_COUNT_HI_MASK 0x0F // Bits [3:0] TRQ_COUNT[11:8]

    // ── Microstep mode enumeration ──────────────────────────────────────────

    typedef enum
    {
        DRV8434S_STEP_FULL = 0,  // 100% current, full step
        DRV8434S_STEP_1_2 = 1,   // 1/2  step (non-circular)
        DRV8434S_STEP_1_4 = 2,   // 1/4  step
        DRV8434S_STEP_1_8 = 3,   // 1/8  step
        DRV8434S_STEP_1_16 = 4,  // 1/16 step
        DRV8434S_STEP_1_32 = 5,  // 1/32 step
        DRV8434S_STEP_1_64 = 6,  // 1/64 step
        DRV8434S_STEP_1_128 = 7, // 1/128 step
        DRV8434S_STEP_1_256 = 8, // 1/256 step
        DRV8434S_STEP_1_2_NC = 9 // 1/2 step (non-circular, alt)
    } drv8434s_microstep_t;

    // ── Decay mode enumeration (CTRL2 bits [2:0]) ──────────────────────────

    typedef enum
    {
        DRV8434S_DECAY_SLOW = 0,
        DRV8434S_DECAY__SLOW_DEC30 = 1,
        DRV8434S_DECAY__SLOW_DEC60 = 2,
        DRV8434S_DECAY_SLOW_FAST = 3,
        DRV8434S_DECAY_MIX30 = 4,
        DRV8434S_DECAY_MIX60 = 5,
        DRV8434S_DECAY_SMART_TUNE_DYNAMIC = 6,
        DRV8434S_DECAY_SMART_TUNE_RIPPLE = 7,
    } drv8434s_decay_t;

    // ── Callback typedefs (unchanged) ───────────────────────────────────────

    typedef int (*drv8434s_spi_xfer_t)(void *user_ctx, const uint8_t *tx, uint8_t *rx, size_t len);
    typedef void (*drv8434s_cs_control_t)(void *user_ctx, bool asserted);
    typedef void (*drv8434s_reset_t)(void *user_ctx, bool asserted);
    typedef void (*drv8434s_delay_ms_t)(void *user_ctx, unsigned ms);
    typedef void (*drv8434s_delay_us_t)(void *user_ctx, unsigned us);

    // ── Configuration & device context ──────────────────────────────────────

    typedef struct
    {
        void *user_ctx;               // passed back to every callback
        drv8434s_spi_xfer_t spi_xfer; // full-duplex SPI transfer
        drv8434s_cs_control_t cs;     // assert / deassert chip-select (active low)
        drv8434s_reset_t reset;       // optional reset control (can be NULL)
        drv8434s_delay_ms_t delay_ms; // optional delay function (can be NULL)
        drv8434s_delay_us_t delay_us; // optional µs delay (can be NULL, used
                                      // for CS hold time & step edge timing)
    } drv8434s_config_t;

    typedef struct
    {
        drv8434s_config_t cfg;
        uint8_t reg_cache[DRV8434S_NUM_REGS]; // shadow copy 0x00 – 0x0B
    } drv8434s_t;

    // ═══════════════════════════════════════════════════════════════════════
    //  Minimal API (unchanged from original driver)
    // ═══════════════════════════════════════════════════════════════════════

    // Initialize device context.  Returns true on success.
    bool drv8434s_init(drv8434s_t *dev, const drv8434s_config_t *cfg);

    // Probe the device: performs a simple SPI transaction and returns true
    // if the device responded (i.e. MISO returned something other than 0xFF).
    bool drv8434s_probe(drv8434s_t *dev);

    // Reset the device using the reset callback (if provided).
    void drv8434s_reset(drv8434s_t *dev, unsigned reset_ms);

    // Send an arbitrary SPI command and read back rx_len bytes.
    bool drv8434s_probe_cmd(drv8434s_t *dev, const uint8_t *tx, size_t tx_len,
                            uint8_t *rx, size_t rx_len);

    // ═══════════════════════════════════════════════════════════════════════
    //  Register-level API
    // ═══════════════════════════════════════════════════════════════════════

    // Write a single 8-bit value to a register.  Returns true on success.
    bool drv8434s_write_reg(drv8434s_t *dev, uint8_t reg, uint8_t value);

    // Read a single register and return its 8-bit value in *value.
    // Unlike the generic drv8434s_read_register(), this uses the proper
    // DRV8434S SPI read protocol (bit 7 = 1).
    bool drv8434s_read_reg(drv8434s_t *dev, uint8_t reg, uint8_t *value);

    // Read-modify-write: reads current value, clears bits in mask, ORs in
    // (value & mask), then writes back.  Returns true on success.
    bool drv8434s_modify_reg(drv8434s_t *dev, uint8_t reg,
                             uint8_t mask, uint8_t value);

    // ═══════════════════════════════════════════════════════════════════════
    //  Motor-control helpers
    // ═══════════════════════════════════════════════════════════════════════

    // Enable motor outputs (sets EN_OUT in CTRL2).
    bool drv8434s_enable(drv8434s_t *dev);

    // Disable motor outputs (clears EN_OUT in CTRL2).
    bool drv8434s_disable(drv8434s_t *dev);

    // Set micro-step resolution (CTRL3 bits [3:0]).
    bool drv8434s_set_microstep(drv8434s_t *dev, drv8434s_microstep_t mode);

    // Set torque / full-scale current DAC (CTRL1 bits [6:3], range 0–15).
    // 0 = 100% full-scale current, 15 = 25% (see Table 7-6).
    bool drv8434s_set_torque(drv8434s_t *dev, uint8_t trq);

    // Set decay mode (CTRL2 bits [2:0]).
    bool drv8434s_set_decay(drv8434s_t *dev, drv8434s_decay_t decay);

    // Enable SPI step/direction mode by setting SPI_STEP and SPI_DIR in
    // CTRL3.  After this call, stepping and direction are controlled via
    // the STEP (bit 6) and DIR (bit 7) bits rather than hardware pins.
    bool drv8434s_set_spi_step_mode(drv8434s_t *dev);

    // Set direction via DIR bit in CTRL3 (0 = forward, 1 = reverse).
    // Requires SPI_DIR mode to be enabled (drv8434s_set_spi_step_mode).
    bool drv8434s_set_spi_dir(drv8434s_t *dev, bool reverse);

    // Issue a single SPI-controlled step (writes STEP bit in CTRL3).
    // The bit is self-clearing on the device.
    // Requires SPI_STEP mode to be enabled (drv8434s_set_spi_step_mode).
    bool drv8434s_spi_step(drv8434s_t *dev);

    // ═══════════════════════════════════════════════════════════════════════
    //  Stall detection helpers
    // ═══════════════════════════════════════════════════════════════════════

    // Enable hardware stall detection.
    //   - Sets EN_STL (CTRL5 bit 4) to enable the stall comparator.
    //   - Sets STL_REP (CTRL5 bit 3) so stall faults assert nFAULT.
    // After enabling, a stall is detected when TRQ_COUNT falls below
    // the STALL_TH threshold, latching STALL, STL and FAULT bits.
    bool drv8434s_enable_stall_detection(drv8434s_t *dev);

    // Disable hardware stall detection (clears EN_STL in CTRL5).
    bool drv8434s_disable_stall_detection(drv8434s_t *dev);

    // Set the 12-bit stall threshold (STALL_TH).
    //   Low 8 bits  → CTRL6 (0x08) STALL_TH[7:0]
    //   High 4 bits → CTRL7 (0x09) bits [3:0] STALL_TH[11:8]
    // When TRQ_COUNT falls below this value, a stall fault is triggered.
    bool drv8434s_set_stall_threshold(drv8434s_t *dev, uint16_t threshold);

    // Start the stall learning process.  The device briefly stalls the motor
    // to learn the ideal STALL_TH value.  Poll drv8434s_is_stall_learned()
    // to know when learning is complete; STALL_TH is updated automatically.
    bool drv8434s_start_stall_learning(drv8434s_t *dev);

    // Check whether stall learning has completed (STL_LRN_OK in DIAG2).
    // Returns true if the learning finished successfully.
    bool drv8434s_is_stall_learned(drv8434s_t *dev);

    // ═══════════════════════════════════════════════════════════════════════
    //  Diagnostics & fault helpers
    // ═══════════════════════════════════════════════════════════════════════

    // ── Fault information structure ────────────────────────────────────────
    //
    // Populated by drv8434s_diagnose_fault() with a snapshot of all three
    // diagnostic registers plus a convenience flag indicating whether any
    // fault bit is active.

    typedef struct
    {
        uint8_t fault;   // FAULT register (0x00)
        uint8_t diag1;   // DIAG1  register (0x01) – OCP per-FET detail
        uint8_t diag2;   // DIAG2  register (0x02) – OTW/OTS/stall/open-load
        bool is_faulted; // true when FAULT bit 7 (global fault) is set
    } drv8434s_fault_info_t;

    // ── Recovery outcome enumeration ──────────────────────────────────────

    typedef enum
    {
        DRV8434S_RECOVER_OK = 0,        // No fault was active
        DRV8434S_RECOVER_CLR_OK,        // Cleared via CLR_FLT (soft clear)
        DRV8434S_RECOVER_RESET_OK,      // Required hardware reset to clear
        DRV8434S_RECOVER_STILL_FAULTED, // Faults persist after all attempts
        DRV8434S_RECOVER_SPI_ERROR,     // SPI communication failed
    } drv8434s_recover_result_t;

    // Read the FAULT register into *faults.  Returns true on success.
    bool drv8434s_read_fault(drv8434s_t *dev, uint8_t *faults);

    // Clear latched faults by writing CLR_FLT in CTRL4.
    bool drv8434s_clear_faults(drv8434s_t *dev);

    // Read the instantaneous torque-DAC count from CTRL8 (TRQ_COUNT).
    bool drv8434s_read_torque_count(drv8434s_t *dev, uint8_t *trq_count);

    // Read all registers into a caller-supplied array.
    // Useful for debug dumps.
    bool drv8434s_read_all_regs(drv8434s_t *dev, uint8_t regs[DRV8434S_NUM_REGS]);

    // Read FAULT, DIAG1 and DIAG2 into a drv8434s_fault_info_t.
    // Returns true on success (even if faults are present).
    bool drv8434s_diagnose_fault(drv8434s_t *dev, drv8434s_fault_info_t *info);

    // Attempt to recover from a fault condition.
    //
    // Strategy (two-stage):
    //   1. Write CLR_FLT in CTRL4 and re-read FAULT.  If clear → done.
    //   2. If faults persist and a hardware reset callback is available,
    //      perform a full hardware reset (nRST toggle) and re-read FAULT.
    //
    // On entry *info is populated with the pre-recovery fault snapshot.
    // On exit  *info is updated with the post-recovery fault state.
    //
    // @param dev       Device handle (must be initialised).
    // @param info      Receives pre- and post-recovery fault diagnostics.
    // @param reset_ms  Duration to hold nRST low during hardware reset
    //                  (ignored when the reset callback is NULL).
    // @return          Recovery outcome code.
    drv8434s_recover_result_t drv8434s_recover_from_fault(
        drv8434s_t *dev, drv8434s_fault_info_t *info, unsigned reset_ms);

#ifdef __cplusplus
}
#endif
#endif // DRV8434S_H
