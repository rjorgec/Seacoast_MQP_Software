#ifndef DRV8434S_H
#define DRV8434S_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
// #define TUNING                                                               \
//   1 // Enable detailed torque buffer prints in drv8434s.c for tuning
//        purposes.  Comment out for cleaner logs.

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

#define DRV8434S_CTRL1_TRQ_DAC_MASK 0xF0 // Bits [7:4] (datasheet Table 7-22)
#define DRV8434S_CTRL1_TRQ_DAC_SHIFT 4
#define DRV8434S_CTRL1_OL_MODE (1u << 1) // Open-load mode select

// ── CTRL2 register (0x04) bit masks ─────────────────────────────────────

#define DRV8434S_CTRL2_EN_OUT (1u << 7) // 1 = outputs enabled
#define DRV8434S_CTRL2_TOFF_MASK                                               \
  0x18 // Bits [4:3] PWM off-time (datasheet Table 7-23)
#define DRV8434S_CTRL2_TOFF_SHIFT 3
#define DRV8434S_CTRL2_DECAY_MASK 0x07 // Bits [2:0] decay mode
#define DRV8434S_CTRL2_DECAY_SHIFT 0

// ── CTRL3 register (0x05) bit masks ─────────────────────────────────────

#define DRV8434S_CTRL3_DIR (1u << 7) // Direction control (when SPI_DIR=1)
#define DRV8434S_CTRL3_STEP                                                    \
  (1u << 6) // Step trigger (when SPI_STEP=1, rising edge)
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

// Datasheet Table 7-24: MICROSTEP_MODE[3:0] in CTRL3 bits [3:0]
typedef enum {
  DRV8434S_STEP_FULL_100 = 0, // 0000 = Full step (2-phase), 100% current
  DRV8434S_STEP_FULL_71 = 1,  // 0001 = Full step (2-phase), 71% current
  DRV8434S_STEP_1_2_NC = 2,   // 0010 = Non-circular 1/2 step
  DRV8434S_STEP_1_2 = 3,      // 0011 = 1/2 step
  DRV8434S_STEP_1_4 = 4,      // 0100 = 1/4 step
  DRV8434S_STEP_1_8 = 5,      // 0101 = 1/8 step
  DRV8434S_STEP_1_16 = 6,     // 0110 = 1/16 step (default)
  DRV8434S_STEP_1_32 = 7,     // 0111 = 1/32 step
  DRV8434S_STEP_1_64 = 8,     // 1000 = 1/64 step
  DRV8434S_STEP_1_128 = 9,    // 1001 = 1/128 step
  DRV8434S_STEP_1_256 = 10,   // 1010 = 1/256 step
                              // 1011–1111 = Reserved
} drv8434s_microstep_t;

// ── Decay mode enumeration (CTRL2 bits [2:0]) ──────────────────────────

typedef enum {
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

typedef int (*drv8434s_spi_xfer_t)(void *user_ctx, const uint8_t *tx,
                                   uint8_t *rx, size_t len);
typedef void (*drv8434s_cs_control_t)(void *user_ctx, bool asserted);
typedef void (*drv8434s_reset_t)(void *user_ctx, bool asserted);
typedef void (*drv8434s_delay_ms_t)(void *user_ctx, unsigned ms);
typedef void (*drv8434s_delay_us_t)(void *user_ctx, unsigned us);

// ── Configuration & device context ──────────────────────────────────────

typedef struct {
  void *user_ctx;               // passed back to every callback
  drv8434s_spi_xfer_t spi_xfer; // full-duplex SPI transfer
  drv8434s_cs_control_t cs;     // assert / deassert chip-select (active low)
  drv8434s_reset_t reset;       // optional reset control (can be NULL)
  drv8434s_delay_ms_t delay_ms; // optional delay function (can be NULL)
  drv8434s_delay_us_t delay_us; // optional µs delay (can be NULL, used
                                // for CS hold time & step edge timing)
} drv8434s_config_t;

typedef struct {
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
bool drv8434s_modify_reg(drv8434s_t *dev, uint8_t reg, uint8_t mask,
                         uint8_t value);

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

typedef struct {
  uint8_t fault;   // FAULT register (0x00)
  uint8_t diag1;   // DIAG1  register (0x01) – OCP per-FET detail
  uint8_t diag2;   // DIAG2  register (0x02) – OTW/OTS/stall/open-load
  bool is_faulted; // true when FAULT bit 7 (global fault) is set
} drv8434s_fault_info_t;

// ── Recovery outcome enumeration ──────────────────────────────────────

typedef enum {
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

// ── Fault configuration flags (for drv8434s_set_fault_config) ───────────
//
// These flags control which fault sources are enabled and how they are
// reported on the nFAULT pin.  Bits [3:0] map to CTRL4 bits [3:0];
// bit 4 maps to CTRL5 STL_REP.
//
// Pass a bitwise-OR of the desired flags to drv8434s_set_fault_config()
// or drv8434s_chain_set_fault_config().  Bits that are *not* set will
// be cleared in the corresponding register fields.

#define DRV8434S_FAULT_CFG_OTW_REP                                             \
  (1u << 0) // Report over-temp warning on nFAULT
#define DRV8434S_FAULT_CFG_OTSD_AUTO (1u << 1) // OTS auto-retry (vs latched)
#define DRV8434S_FAULT_CFG_OCP_AUTO (1u << 2)  // OCP auto-retry (vs latched)
#define DRV8434S_FAULT_CFG_EN_OL (1u << 3)     // Enable open-load detection
#define DRV8434S_FAULT_CFG_STL_REP (1u << 4)   // Report stall on nFAULT

// Configure which faults are enabled / reported on nFAULT.
//
// flags is a bitwise-OR of DRV8434S_FAULT_CFG_* values.  Bits that are
// not set will disable the corresponding fault source or report.
//
// Affects CTRL4 bits [3:0] (EN_OL, OCP_MODE, OTSD_MODE, OTW_REP) and
// CTRL5 bit 3 (STL_REP).
bool drv8434s_set_fault_config(drv8434s_t *dev, uint8_t flags);

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
drv8434s_recover_result_t
drv8434s_recover_from_fault(drv8434s_t *dev, drv8434s_fault_info_t *info,
                            unsigned reset_ms);

// ═══════════════════════════════════════════════════════════════════════
//  SPI Daisy-Chain API  (Section 7.5.1.3 of DRV8434S datasheet)
// ═══════════════════════════════════════════════════════════════════════
//
// When multiple DRV8434S devices are wired in series (MOSI→SDI_1→SDO_1→
// SDI_2→SDO_2→…→SDO_N→MISO), a single CS assertion transfers one frame
// that addresses all devices simultaneously.
//
// TX frame (2 + 2·N bytes):
//   [HDR1][HDR2][A_N][A_{N-1}]…[A_1][D_N][D_{N-1}]…[D_1]
//
// RX frame (2 + 2·N bytes, received in parallel with TX):
//   [S_N][S_{N-1}]…[S_1][HDR1][HDR2][R_N][R_{N-1}]…[R_1]
//
// HDR1 = 10[N5:N0]  – "10" prefix, lower 6 bits = device count
// HDR2 = 10xxxxxx   – "10" prefix, lower bits are don't-care
// S_k  = 11[fault]  – "11" prefix, lower 6 bits = fault flags for device k
// A_k / D_k         – same 2-byte format as single-device mode
// R_k               – register data returned by device k
//
// Device indices in this API are 0-based, where 0 is the device whose
// SDI is connected directly to the MCU MOSI.

// Maximum devices per chain (datasheet limit = 63; practical compile-time cap).
#define DRV8434S_CHAIN_MAX_DEVICES 8u

// Maximum TX/RX buffer size for a fully-populated chain.
#define DRV8434S_CHAIN_MAX_FRAME (2u + 2u * DRV8434S_CHAIN_MAX_DEVICES)

// Header byte bit patterns.
#define DRV8434S_CHAIN_HDR_PREFIX 0x80u // bits[7:6] = "10"
#define DRV8434S_CHAIN_HDR_N_MASK 0x3Fu // HDR1 bits[5:0] = device count
#define DRV8434S_CHAIN_HDR2_CLR                                                \
  0x20u // HDR2 bit 5 = global fault clear on nSCS↑

// Status byte bit patterns.
#define DRV8434S_CHAIN_STAT_PREFIX 0xC0u // bits[7:6] = "11"
#define DRV8434S_CHAIN_STAT_MASK 0x3Fu   // bits[5:0] = fault flags

// ── Daisy-chain types ────────────────────────────────────────────────────

// Configuration shared by all devices in one daisy chain.
typedef struct {
  void *user_ctx;               // Passed to every callback
  drv8434s_spi_xfer_t spi_xfer; // Full-duplex SPI; one call covers all devices
  drv8434s_cs_control_t cs;     // Single CS that spans the whole chain frame
  drv8434s_delay_ms_t delay_ms; // Optional ms delay (can be NULL)
  drv8434s_delay_us_t delay_us; // Optional µs delay (recommended; used for
                                // CS hold time and step edge timing)
  uint8_t n_devices; // Devices in chain (1 – DRV8434S_CHAIN_MAX_DEVICES)
} drv8434s_chain_config_t;

// Per-device fault status decoded from the RX status bytes.
typedef struct {
  uint8_t raw;     // Raw status byte from the chain frame
  bool is_faulted; // True if any fault bit (bits[5:0]) is set
} drv8434s_chain_dev_status_t;

// Chain context.  Owns per-device register shadow caches.
typedef struct {
  drv8434s_chain_config_t cfg;
  uint8_t reg_cache[DRV8434S_CHAIN_MAX_DEVICES][DRV8434S_NUM_REGS];
} drv8434s_chain_t;

// ── Daisy-chain lifecycle ────────────────────────────────────────────────

// Initialise a chain context from cfg.  Returns false if cfg is invalid
// or n_devices is out of range (0 or > DRV8434S_CHAIN_MAX_DEVICES).
bool drv8434s_chain_init(drv8434s_chain_t *chain,
                         const drv8434s_chain_config_t *cfg);

// Send a chain frame with the HDR2 global-CLR bit set (bit 5 = 1).
// All devices in the chain clear their fault registers on the rising
// edge of nSCS at the end of this frame (datasheet Fig 7-27, HDR2 CLR).
// This is the fastest recovery from SPI_ERROR faults caused by VM
// power-on transients, because it does NOT require a register read-
// modify-write and works even when the SPI state machine is out of
// sync.  All devices receive a NOP (read FAULT) alongside the CLR.
// Returns false only on SPI transfer failure.
bool drv8434s_chain_global_clear_faults(drv8434s_chain_t *chain);

// ── Per-device register access (targeted – all others receive NOP) ───────
//
// NOP = read FAULT register; harmless side effect, no state change.
// status_out: optional array of n_devices entries; receives the status
// byte for every device captured during the frame.  Pass NULL to ignore.

// Write one register on device dev_idx; all others get a NOP.
bool drv8434s_chain_write_reg(drv8434s_chain_t *chain, uint8_t dev_idx,
                              uint8_t reg, uint8_t value,
                              drv8434s_chain_dev_status_t *status_out);

// Read one register from device dev_idx; all others get a NOP.
bool drv8434s_chain_read_reg(drv8434s_chain_t *chain, uint8_t dev_idx,
                             uint8_t reg, uint8_t *value,
                             drv8434s_chain_dev_status_t *status_out);

// Read-modify-write one register on device dev_idx.
bool drv8434s_chain_modify_reg(drv8434s_chain_t *chain, uint8_t dev_idx,
                               uint8_t reg, uint8_t mask, uint8_t value,
                               drv8434s_chain_dev_status_t *status_out);

// ── Broadcast register access (one frame, all devices) ───────────────────

// Write the same register address with per-device values in one frame.
// values[k] is written to device k (0-based).
bool drv8434s_chain_write_all(drv8434s_chain_t *chain, uint8_t reg,
                              const uint8_t *values,
                              drv8434s_chain_dev_status_t *status_out);

// Read the same register from every device in one frame.
// values_out[k] receives the report byte for device k (0-based).
bool drv8434s_chain_read_all(drv8434s_chain_t *chain, uint8_t reg,
                             uint8_t *values_out,
                             drv8434s_chain_dev_status_t *status_out);

// ── Per-device motor-control helpers ─────────────────────────────────────
//
// These target a single device in the chain; all other devices in the
// same SPI frame receive a NOP (read FAULT).

// Enable motor outputs on a single device (sets EN_OUT in CTRL2).
bool drv8434s_chain_enable(drv8434s_chain_t *chain, uint8_t dev_idx,
                           drv8434s_chain_dev_status_t *status_out);

// Disable motor outputs on a single device (clears EN_OUT in CTRL2).
bool drv8434s_chain_disable(drv8434s_chain_t *chain, uint8_t dev_idx,
                            drv8434s_chain_dev_status_t *status_out);

// Issue a single SPI step to one device.  STEP bit is self-clearing;
// the shadow cache is updated so the next call produces a clean edge.
// Requires SPI step mode on that device (drv8434s_chain_set_spi_step_mode).
bool drv8434s_chain_step(drv8434s_chain_t *chain, uint8_t dev_idx,
                         drv8434s_chain_dev_status_t *status_out);

// Set direction for a single device.  reverse=false → forward, true → reverse.
// Requires SPI_DIR mode on that device.
bool drv8434s_chain_set_dir(drv8434s_chain_t *chain, uint8_t dev_idx,
                            bool reverse,
                            drv8434s_chain_dev_status_t *status_out);

// Set microstep resolution for a single device (CTRL3 bits [3:0]).
bool drv8434s_chain_set_microstep(drv8434s_chain_t *chain, uint8_t dev_idx,
                                  drv8434s_microstep_t mode,
                                  drv8434s_chain_dev_status_t *status_out);

// Set torque / full-scale current DAC for a single device (CTRL1 bits [6:3]).
// trq range 0–15: 0 = 100% full-scale, 15 = 25%.
bool drv8434s_chain_set_torque(drv8434s_chain_t *chain, uint8_t dev_idx,
                               uint8_t trq,
                               drv8434s_chain_dev_status_t *status_out);

// Enable SPI step/direction mode for a single device (sets SPI_STEP and
// SPI_DIR in CTRL3).  Must be called before chain_step / chain_set_dir
// will affect the device.
bool drv8434s_chain_set_spi_step_mode(drv8434s_chain_t *chain, uint8_t dev_idx,
                                      drv8434s_chain_dev_status_t *status_out);

// Read the 12-bit torque count (TRQ_COUNT) from a single device.
// Combines CTRL8 (low 8 bits) and CTRL9 bits [3:0] (high 4 bits).
bool drv8434s_chain_read_torque_count(drv8434s_chain_t *chain, uint8_t dev_idx,
                                      uint16_t *torque_out,
                                      drv8434s_chain_dev_status_t *status_out);

// Clear latched faults on a single device (sets CLR_FLT in CTRL4).
bool drv8434s_chain_clear_faults(drv8434s_chain_t *chain, uint8_t dev_idx,
                                 drv8434s_chain_dev_status_t *status_out);

// Configure which faults are enabled / reported on nFAULT for a single
// device in the chain.  flags is a bitwise-OR of DRV8434S_FAULT_CFG_*
// values (see single-device drv8434s_set_fault_config for details).
bool drv8434s_chain_set_fault_config(drv8434s_chain_t *chain, uint8_t dev_idx,
                                     uint8_t flags,
                                     drv8434s_chain_dev_status_t *status_out);

// ── Non-blocking motion engine ──────────────────────────────────────────
//
// The motion engine manages concurrent stepping of multiple motors in a
// daisy chain.  Each device has independent motion state (target, torque
// limit, step count).  A single drv8434s_motion_tick() call advances all
// active motors in ONE SPI chain frame, making it efficient and suitable
// for timer-interrupt contexts (e.g. Pico SDK add_repeating_timer_us).
//
// Usage:
//   1. drv8434s_motion_init()  — bind to a chain, set done callback
//   2. drv8434s_motion_start() — queue a motion on a device (non-blocking)
//   3. Timer calls drv8434s_motion_tick() at the desired step rate
//   4. done_cb fires when each motor completes or faults

// Stop reason codes.
typedef enum {
  DRV8434S_MOTION_OK = 0,       // Reached target position
  DRV8434S_MOTION_TORQUE_LIMIT, // Stopped: torque threshold exceeded
  DRV8434S_MOTION_FAULT,        // Stopped: device fault detected
  DRV8434S_MOTION_SPI_ERROR,    // Stopped: SPI communication failure
  DRV8434S_MOTION_CANCELLED,    // Stopped: cancelled by caller
} drv8434s_motion_stop_reason_t;

// Result structure populated when a motion job completes.
typedef struct {
  int32_t steps_requested;    // Echo of the requested step count
  int32_t steps_achieved;     // Actual steps completed (signed: +fwd, -rev)
  uint16_t last_torque_count; // Last TRQ_COUNT reading
  drv8434s_motion_stop_reason_t reason; // Why the motion ended
} drv8434s_motion_result_t;

// Completion callback — invoked from drv8434s_motion_tick() context when
// a motor finishes (reached target, torque limit, fault, or cancel).
typedef void (*drv8434s_motion_done_cb_t)(
    void *ctx, uint8_t dev_idx, const drv8434s_motion_result_t *result);

// Rolling-average depth for per-job torque readings.
// Smaller rolling history reduces latency for torque surge detection.
// 3 samples is enough to filter noise while still reacting quickly.
#define DRV8434S_MOTION_TRQ_BUF_SIZE 3u

// Per-device motion job state.  Managed internally by the motion engine.
typedef struct {
  int32_t steps_remaining; // Absolute count, decrements each tick
  int32_t steps_achieved;  // Signed accumulator
  int32_t steps_requested; // Original signed request
  uint16_t torque_limit;   // 0 = disabled
  uint16_t last_torque_count;
  drv8434s_motion_stop_reason_t reason;
  bool active;
  bool reverse;
  // Torque rolling-average buffer
  uint16_t trq_buf[DRV8434S_MOTION_TRQ_BUF_SIZE];
  uint8_t trq_buf_idx;
  uint8_t trq_buf_count;
} drv8434s_motion_job_t;

// Motion engine context.  Owns per-device job state for one chain.
typedef struct {
  drv8434s_chain_t *chain;
  drv8434s_motion_done_cb_t done_cb;
  void *done_ctx;
  uint8_t
      torque_sample_div; // Sample torque every N ticks (0 or 1 = every tick)
  uint8_t torque_tick_count; // Internal counter
  drv8434s_motion_job_t jobs[DRV8434S_CHAIN_MAX_DEVICES];
} drv8434s_motion_t;

// Initialise the motion engine, binding it to a chain context.
// done_cb is called (from tick context) when any motor completes.
// done_cb and done_ctx may be NULL if polling is preferred.
// torque_sample_div controls how often torque is read (1 = every tick,
// 10 = every 10th tick); 0 is treated as 1.
bool drv8434s_motion_init(drv8434s_motion_t *motion, drv8434s_chain_t *chain,
                          drv8434s_motion_done_cb_t done_cb, void *done_ctx,
                          uint8_t torque_sample_div);

// Start a non-blocking motion on device dev_idx.
// target_steps > 0 → forward, < 0 → reverse, 0 → no-op.
// torque_limit: trip threshold for TRQ_COUNT rolling average (0 = disabled).
// Returns false if dev_idx is invalid or the device is already moving.
// The caller must start a periodic timer that calls drv8434s_motion_tick().
bool drv8434s_motion_start(drv8434s_motion_t *motion, uint8_t dev_idx,
                           int32_t target_steps, uint16_t torque_limit);

// Advance all active motors by one step in a single SPI chain frame.
// Call this from a repeating timer callback at the desired step rate.
// Returns false only on catastrophic SPI failure (all active jobs aborted).
bool drv8434s_motion_tick(drv8434s_motion_t *motion);

// Cancel motion on a specific device.  Populates *result if non-NULL.
// The done_cb is NOT invoked for cancellations.
bool drv8434s_motion_cancel(drv8434s_motion_t *motion, uint8_t dev_idx,
                            drv8434s_motion_result_t *result);

// Returns true if any device has an active motion job.
bool drv8434s_motion_is_busy(const drv8434s_motion_t *motion);

#ifdef __cplusplus
}
#endif
#endif // DRV8434S_H