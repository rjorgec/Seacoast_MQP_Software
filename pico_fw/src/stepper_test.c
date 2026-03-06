/**
 * stepper_test.c — Standalone DRV8434S stepper bring-up test.
 *
 * Exercises the DRV8434S daisy-chain driver with the SPI frame protocol
 * described in drv8434s.h (Section 7.5.1.3):
 *
 *   SPI Mode 1 (CPOL=0, CPHA=1), MSB-first, 8-bit frames.
 *
 *   Single-device frame (16 bits on wire):
 *     Byte 0: [0][W][A4][A3][A2][A1][A0][0]
 *     Byte 1: [D7][D6][D5][D4][D3][D2][D1][D0]
 *
 *   Daisy-chain frame (2 + 2·N bytes, built by drv8434s_chain_* API):
 *     TX: [HDR1][HDR2][A_{N-1}]…[A_0][D_{N-1}]…[D_0]
 *     RX: [S_{N-1}]…[S_0][HDR1][HDR2][R_{N-1}]…[R_0]
 *
 * The spi_xfer callback receives the fully-built frame from the driver
 * and must perform a raw full-duplex SPI transfer of the given length
 * while CS is asserted by the driver's chain_xfer() internally.
 *
 * Test behaviour: configures the chain, probes each device, enables
 * SPI step/direction mode, then runs a forward/reverse agitation loop
 * on device 0.
 */

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

#include "board_pins.h"
#include "drivers/drv8434s/drv8434s.h"

/* ── SPI platform callbacks ────────────────────────────────────────────────
 *
 * These callbacks are invoked by the drv8434s chain driver during every
 * SPI frame transfer (including from the repeating-timer step tick).
 * They MUST be fast and free of printf / blocking calls.
 *
 * The chain driver builds the complete daisy-chain frame (2 + 2·N bytes
 * with HDR1, HDR2, per-device address and data slots) and passes it as
 * the tx buffer.  The callback's only job is to clock those bytes out
 * and capture the simultaneous rx bytes.  CS assertion/deassertion and
 * inter-frame timing are handled by the driver's chain_xfer().
 */

static int stepper_spi_xfer(void *ctx, const uint8_t *tx, uint8_t *rx, size_t len)
{
    spi_inst_t *spi = (spi_inst_t *)ctx;
    spi_write_read_blocking(spi, tx, rx, len);
    return 0;
}

static void stepper_cs(void *ctx, bool asserted)
{
    (void)ctx;
    /* DRV8434S CS is active-low: assert = drive low, deassert = drive high. */
    gpio_put((uint)DRV8434S_CS_GPIO, asserted ? 0u : 1u);
}

static void stepper_delay_us(void *ctx, unsigned us)
{
    (void)ctx;
    /* MUST use busy_wait_us_32, NOT sleep_us.
     *
     * sleep_us() internally calls best_effort_wfe_or_timeout() which issues
     * a __wfe() (Wait For Event) instruction.  This callback is invoked from
     * the repeating-timer alarm IRQ context (add_repeating_timer_us).  Calling
     * __wfe() inside that IRQ deadlocks: the alarm hardware needed to generate
     * the wake event is the same one we are currently servicing, so the wakeup
     * event never fires and the CPU hangs forever, starving the main loop.
     *
     * busy_wait_us_32() spins counting CPU cycles with no timer hardware
     * dependency and is safe to call from any IRQ context. */
    busy_wait_us_32(us);
}

static void stepper_delay_ms(void *ctx, unsigned ms)
{
    (void)ctx;
    sleep_ms(ms);
}

/* ── Globals ───────────────────────────────────────────────────────────────── */

static drv8434s_chain_t s_chain;
static drv8434s_motion_t s_motion;
static struct repeating_timer s_step_timer;
static bool s_step_timer_active;
static bool s_need_new_motion;
static bool s_reverse_next;
static uint32_t s_last_done_ms;

/* Counters updated from the repeating-timer IRQ context (volatile so the
 * main loop sees writes without being optimised away).  Used to confirm
 * the timer is firing and to detect STATUS-byte fault hits per run. */
static volatile uint32_t s_tick_count      = 0u;
static volatile uint32_t s_fault_tick_count = 0u;

/* ── Motion completion callback ────────────s────────────────────────────────── */

static void stepper_motion_done(void *ctx, uint8_t dev_idx,
                                const drv8434s_motion_result_t *res)
{
    (void)ctx;
    printf("Stepper[%u]: done — %li of %li steps, torque=%u, reason=%u\n",
           dev_idx,
           (long)res->steps_achieved,
           (long)res->steps_requested,
           res->last_torque_count,
           (unsigned)res->reason);
    s_last_done_ms = to_ms_since_boot(get_absolute_time());
    s_need_new_motion = true;
    /* toggle direction for next job */
    s_reverse_next = !s_reverse_next;
}

/* ── Step timer (called at AGITATOR_STEP_DELAY_US interval) ────────────────
 *
 * drv8434s_motion_tick() advances all active motors by one step in a
 * single SPI chain frame.  This callback MUST be lightweight — the
 * spi_xfer callback above is intentionally printf-free for this reason.
 */

static bool step_timer_callback(struct repeating_timer *t)
{
    (void)t;
    ++s_tick_count; /* IRQ-safe: only written here, main loop only reads */
    drv8434s_motion_tick(&s_motion);

    if (!drv8434s_motion_is_busy(&s_motion))
    {
        s_step_timer_active = false;
        return false; /* cancel timer */
    }
    return true; /* keep running */
}

static bool ensure_step_timer_running(uint32_t delay_us)
{
    if (s_step_timer_active)
        return true;
    if (!add_repeating_timer_us(-(int64_t)delay_us,
                                step_timer_callback, NULL, &s_step_timer))
    {
        return false;
    }
    s_step_timer_active = true;
    return true;
}

/* ── Raw SPI frame dump ─────────────────────────────────────────────────────
 *
 * Shows the exact bytes the Pico sends (TX) and receives (RX) for a single
 * chain frame.  For N=1 the frame is 4 bytes:
 *   TX: [HDR1][HDR2][ADDR][DATA]
 *   RX: [STATUS][HDR1_echo][HDR2_echo][REPORT]
 *
 * STATUS byte format: 11[UVLO][CPUV][OCP][STL][TF][OL]
 *   bits [7:6] are always "11" on a real device; 0xFF = MISO floating or
 *   all-faults set.
 *
 * Call this helper when RX looks wrong to see exactly what the device is
 * returning before any parsing.
 */
static void spi_raw_frame_dump(const char *label,
                               const uint8_t *tx, const uint8_t *rx,
                               size_t len)
{
    printf("  %s TX[%u]:", label, (unsigned)len);
    for (size_t i = 0; i < len; ++i)
        printf(" %02X", tx[i]);
    printf("\n");
    printf("  %s RX[%u]:", label, (unsigned)len);
    for (size_t i = 0; i < len; ++i)
        printf(" %02X", rx[i]);
    printf("\n");

    if (len >= 4)
    {
        /* For N=1 chain frame, decode the RX fields. */
        uint8_t status = rx[0];
        uint8_t report = rx[len - 1];
        bool valid_prefix = (status & 0xC0u) == 0xC0u;
        printf("  %s STATUS=0x%02X%s  REPORT=0x%02X\n",
               label, status,
               valid_prefix ? " (prefix OK)" : " *** INVALID PREFIX – MISO may be floating ***",
               report);
        if (status == 0xFF && report == 0xFF)
            printf("  %s WARN: all 0xFF – DRV8434S SDO is Hi-Z. "
                   "Check: VM present? SPI_ERROR latched? MISO pull-up present?\n",
                   label);
        else if (!valid_prefix)
            printf("  %s WARN: status byte prefix [7:6] != '11' – SPI mode or "
                   "wiring issue (CPOL/CPHA/MSB order)\n",
                   label);
        if (valid_prefix && (status & 0x3Fu))
        {
            printf("  %s device fault flags set in STATUS: 0x%02X\n",
                   label, status & 0x3Fu);
            if (status & 0x20u)
                printf("    UVLO (motor supply under-voltage)\n");
            if (status & 0x10u)
                printf("    CPUV (charge pump under-voltage)\n");
            if (status & 0x08u)
                printf("    OCP  (over-current)\n");
            if (status & 0x04u)
                printf("    STL  (stall)\n");
            if (status & 0x02u)
                printf("    TF   (over-temperature)\n");
            if (status & 0x01u)
                printf("    OL   (open load)\n");
        }
    }
}

/* ── Low-level raw chain frame probe ────────────────────────────────────────
 *
 * Sends one manually-built chain frame directly to the SPI hardware,
 * bypassing the driver, and dumps both the TX and RX bytes.  Used when
 * the driver API returns 0xFF (device not responding) to diagnose whether
 * the problem is MISO floating, wrong SPI mode, or a device fault state.
 *
 * For N=1 reading register `reg`:
 *   TX: [0x81][0x80][READ_BIT | (reg<<1)][0x00]
 *   RX: [STATUS][HDR1_echo][HDR2_echo][REPORT]
 */
static void raw_probe_once(spi_inst_t *spi, uint8_t reg, const char *label)
{
    const uint8_t N = (uint8_t)DRV8434S_N_DEVICES;
    const size_t frame_len = 2u + 2u * (size_t)N;

    uint8_t tx[2u + 2u * DRV8434S_CHAIN_MAX_DEVICES];
    uint8_t rx[2u + 2u * DRV8434S_CHAIN_MAX_DEVICES];

    for (size_t i = 0; i < frame_len; ++i)
    {
        tx[i] = 0;
        rx[i] = 0xFF;
    }

    /* HDR1: [1][0][N5:N0] */
    tx[0] = (uint8_t)(0x80u | (N & 0x3Fu));
    /* HDR2: [1][0][CLR=0][x:x:x:x:x] */
    tx[1] = 0x80u;
    /* For N=1, device 0 address occupies tx[2], data tx[3]. */
    tx[2] = (uint8_t)(DRV8434S_SPI_READ_BIT |
                      ((reg & DRV8434S_ADDR_MASK) << DRV8434S_ADDR_SHIFT));
    tx[3] = 0x00u;

    gpio_put((uint)DRV8434S_CS_GPIO, 0u); /* assert CS */
    spi_write_read_blocking(spi, tx, rx, frame_len);
    gpio_put((uint)DRV8434S_CS_GPIO, 1u); /* deassert CS */
    sleep_us(5u);

    spi_raw_frame_dump(label, tx, rx, frame_len);
}

/* ── Debug: dump a register read for one device ───────────────────────────── */

static void dump_device_regs(uint8_t dev_idx)
{
    static const char *reg_names[] = {
        "FAULT",
        "DIAG1",
        "DIAG2",
        "CTRL1",
        "CTRL2",
        "CTRL3",
        "CTRL4",
        "CTRL5",
        "CTRL6",
        "CTRL7",
        "CTRL8",
        "CTRL9",
    };
    printf("  dev %u registers:\n", dev_idx);
    for (uint8_t r = 0; r < DRV8434S_NUM_REGS; ++r)
    {
        uint8_t val = 0;
        bool ok = drv8434s_chain_read_reg(&s_chain, dev_idx, r, &val, NULL);
        printf("    0x%02X %-6s = 0x%02X%s\n",
               r, reg_names[r], val, ok ? "" : " (READ FAILED)");
    }
}

/* ── Verify register defaults against datasheet Table 7-15 ─────────────── */

static void verify_defaults(uint8_t dev_idx)
{
    /* Datasheet reset values (Table 7-15 / sections 7.6.2–7.6.13):
     *   FAULT  (0x00) = 0x00   DIAG1  (0x01) = 0x00   DIAG2  (0x02) = 0x00
     *   CTRL1  (0x03) = 0x00   CTRL2  (0x04) = 0x0F   CTRL3  (0x05) = 0x06
     *   CTRL4  (0x06) = 0x30   CTRL5  (0x07) = 0x08   CTRL6  (0x08) = 0x03
     *   CTRL7  (0x09) = 0x20   CTRL8  (0x0A) = 0x00   CTRL9  (0x0B) = 0x00
     *
     * CTRL2 = 0x0F: EN_OUT=0, RSVD=00, TOFF=01(bits[4:3]), DECAY=111(bits[2:0])
     * CTRL3 = 0x06: DIR=0, STEP=0, SPI_DIR=0, SPI_STEP=0, MICROSTEP=0110 (1/16)
     * CTRL4 = 0x30: CLR_FLT=0, LOCK=011, EN_OL=0, OCP_MODE=0, OTSD_MODE=0, OTW_REP=0
     * CTRL5 = 0x08: RSVD=00, STL_LRN=0, EN_STL=0, STL_REP=1, RSVD=000
     * CTRL7 = 0x20: RC_RIPPLE=00, EN_SSC=1, TRQ_SCALE=0, STALL_TH[11:8]=0000
     */
    static const uint8_t expected[DRV8434S_NUM_REGS] = {
        0x00,
        0x00,
        0x00, /* FAULT, DIAG1, DIAG2 */
        0x00, /* CTRL1 */
        0x0F, /* CTRL2 */
        0x06, /* CTRL3 */
        0x30, /* CTRL4 */
        0x08, /* CTRL5 */
        0x03, /* CTRL6 */
        0x20, /* CTRL7 */
        0x00, /* CTRL8 */
        0x00, /* CTRL9 (upper 4 bits = REV_ID, varies by silicon) */
    };
    static const char *reg_names[] = {
        "FAULT",
        "DIAG1",
        "DIAG2",
        "CTRL1",
        "CTRL2",
        "CTRL3",
        "CTRL4",
        "CTRL5",
        "CTRL6",
        "CTRL7",
        "CTRL8",
        "CTRL9",
    };
    /* Skip CTRL8/CTRL9 (read-only live counters) and CTRL9 REV_ID nibble */
    static const uint8_t check_mask[DRV8434S_NUM_REGS] = {
        0xFF,
        0xFF,
        0xFF,
        0xFF,
        0xFF,
        0xFF,
        0xFF,
        0xFF,
        0xFF,
        0xFF,
        0x00,
        0x00,
    };

    uint8_t mismatches = 0;
    printf("  dev %u default verification:\n", dev_idx);
    for (uint8_t r = 0; r < DRV8434S_NUM_REGS; ++r)
    {
        if (check_mask[r] == 0)
            continue;
        uint8_t val = 0;
        if (!drv8434s_chain_read_reg(&s_chain, dev_idx, r, &val, NULL))
        {
            printf("    0x%02X %-6s READ FAILED\n", r, reg_names[r]);
            ++mismatches;
            continue;
        }
        uint8_t masked_val = val & check_mask[r];
        uint8_t masked_exp = expected[r] & check_mask[r];
        if (masked_val != masked_exp)
        {
            printf("    0x%02X %-6s = 0x%02X  EXPECTED 0x%02X  *** MISMATCH ***\n",
                   r, reg_names[r], val, expected[r]);
            ++mismatches;
        }
        else
        {
            printf("    0x%02X %-6s = 0x%02X  OK\n", r, reg_names[r], val);
        }
    }
    if (mismatches == 0)
        printf("  dev %u: all register defaults match datasheet\n", dev_idx);
    else
        printf("  dev %u: %u register mismatch(es) — check power-on-reset state\n",
               dev_idx, mismatches);
}

/* ═══════════════════════════════════════════════════════════════════════════ */

int main(void)
{
    stdio_init_all();
    sleep_ms(7000); /* wait for USB CDC enumeration */
    printf("stepper_test: start\n");

    /* ── Validate SPI pins ────────────────────────────────────────────────── */

    if (DRV8434S_SCK_GPIO < 0 || DRV8434S_MOSI_GPIO < 0 ||
        DRV8434S_MISO_GPIO < 0 || DRV8434S_CS_GPIO < 0)
    {
        printf("stepper_test: DRV8434S SPI pins not set in board_pins.h; aborting\n");
        while (true)
            tight_loop_contents();
    }

    printf("Pins: SCK=%d MOSI=%d MISO=%d CS=%d, N_DEVICES=%d\n",
           DRV8434S_SCK_GPIO,
           DRV8434S_MOSI_GPIO,
           DRV8434S_MISO_GPIO,
           DRV8434S_CS_GPIO,
           DRV8434S_N_DEVICES);

    /* ── SPI hardware init ────────────────────────────────────────────────
     *
     * DRV8434S requires SPI Mode 1 (CPOL=0, CPHA=1), MSB-first, 8-bit
     * frames.  The driver builds 16-bit register frames and multi-byte
     * daisy-chain frames in software; the Pico SPI peripheral transfers
     * them as a contiguous byte stream with software-controlled CS. */

    spi_inst_t *spi = (DRV8434S_SPI_ID == 0) ? spi0 : spi1;
    spi_init(spi, (uint)DRV8434S_SPI_BAUD);
    spi_set_format(spi, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);

    gpio_set_function((uint)DRV8434S_SCK_GPIO, GPIO_FUNC_SPI);
    gpio_set_function((uint)DRV8434S_MOSI_GPIO, GPIO_FUNC_SPI);
    gpio_set_function((uint)DRV8434S_MISO_GPIO, GPIO_FUNC_SPI);
    /* Pull MISO high so it reads 0xFF (not random) when the DRV8434S SDO is
     * in Hi-Z (nSCS deasserted or device not yet driving the line).  This
     * makes "no device present" clearly distinguishable from valid data and
     * prevents indeterminate reads during power-on before VM is stable. */
    gpio_pull_up((uint)DRV8434S_MISO_GPIO);

    /* CS is software-controlled (active-low); start deasserted (high). */
    gpio_init((uint)DRV8434S_CS_GPIO);
    gpio_set_dir((uint)DRV8434S_CS_GPIO, GPIO_OUT);
    gpio_put((uint)DRV8434S_CS_GPIO, 1u);

    /* ── Initialise driver chain context ──────────────────────────────────
     *
     * drv8434s_chain_config_t carries the platform callbacks the driver
     * uses for every SPI frame.  The chain API (drv8434s_chain_*) builds
     * the daisy-chain header + per-device address/data slots internally
     * and passes the assembled buffer to spi_xfer. */

    drv8434s_chain_config_t cfg = {
        .user_ctx = spi,
        .spi_xfer = stepper_spi_xfer,
        .cs = stepper_cs,
        .delay_ms = stepper_delay_ms,
        .delay_us = stepper_delay_us,
        .n_devices = (uint8_t)DRV8434S_N_DEVICES,
    };

    if (!drv8434s_chain_init(&s_chain, &cfg))
    {
        printf("stepper_test: chain init failed\n");
        return 1;
    }

    /* ── Probe with VM power-on fault recovery ────────────────────────────
     *
     * Problem: when VM (motor PSU) is applied near the time of the first
     * SPI frame, the power-on transient can couple noise onto SCLK, causing
     * the DRV8434S to count wrong clock pulses → SPI_ERROR latched in FAULT
     * (bit 6).  With SPI_ERROR active, SDO is not driven reliably → MISO
     * reads 0xFF via the pull-up added above.
     *
     * Recovery uses the datasheet HDR2 global-CLR mechanism (Fig 7-27):
     * HDR2 bit 5 = CLR = 1 clears all fault registers on the rising edge of
     * nSCS.  This works even when the device's SPI state machine is out of
     * sync from a corrupted frame.
     *
     * For each attempt:
     *   1. Raw probe (raw_probe_once) — see exact TX/RX bytes
     *   2. HDR2 global CLR — clears faults on all chain devices
     *   3. Per-device CLR_FLT write to CTRL4 — belt-and-suspenders
     *   4. 10 ms settling delay
     *   5. Driver-API FAULT read to confirm clear
     */
    bool any_ok = false;
    for (int attempt = 0; attempt < 3 && !any_ok; ++attempt)
    {
        printf("stepper_test: probe attempt %d\n", attempt + 1);

        /* Step 1 – raw frame dump of FAULT register read */
        printf("  raw FAULT read (N=%d, frame_len=%d):\n",
               DRV8434S_N_DEVICES, 2 + 2 * DRV8434S_N_DEVICES);
        raw_probe_once(spi, DRV8434S_REG_FAULT, "FAULT");

        /* Step 2 – HDR2 global CLR: clears all device faults on nSCS↑ */
        if (attempt > 0)
        {
            printf("  sending HDR2 global CLR...\n");
            drv8434s_chain_global_clear_faults(&s_chain);
            sleep_ms(5);
        }

        /* Step 3 – per-device CLR_FLT write to CTRL4 = 0xB0
         *   0xB0 = CLR_FLT(bit7)=1 | LOCK(bits6:4)=011 | rest=0
         *   LOCK=011 = unlock value; writing 110 would lock, 011 unlocks.
         *   This does NOT read first — avoids modify-read-write with bad data. */
        if (attempt > 0)
        {
            printf("  writing CLR_FLT to CTRL4...\n");
            for (uint8_t k = 0; k < s_chain.cfg.n_devices; ++k)
            {
                drv8434s_chain_write_reg(&s_chain, k, DRV8434S_REG_CTRL4,
                                         DRV8434S_CTRL4_CLR_FLT | 0x30u,
                                         NULL);
            }
            sleep_ms(10);
        }

        /* Step 4 – read FAULT via driver API to confirm */
        for (uint8_t k = 0; k < s_chain.cfg.n_devices; ++k)
        {
            uint8_t fault_val = 0;
            drv8434s_chain_dev_status_t status = {0};
            if (!drv8434s_chain_read_reg(&s_chain, k, DRV8434S_REG_FAULT,
                                         &fault_val, &status))
            {
                printf("  dev %u: FAULT read failed\n", k);
                continue;
            }
            printf("  dev %u: FAULT=0x%02X  status=0x%02X", k, fault_val, status.raw);
            if (fault_val == 0xFF)
                printf("  *** SDO Hi-Z – VM not present or SPI_ERROR still active\n");
            else if (fault_val & DRV8434S_FAULT_SPI_ERR)
                printf("  SPI_ERROR bit set – retrying\n");
            else
            {
                printf("  OK\n");
                any_ok = true;
            }
        }

        if (!any_ok && attempt < 2)
            sleep_ms(20);
    }

    if (!any_ok)
    {
        printf("stepper_test: no devices responded after %d attempts.\n"
               "  Checklist:\n"
               "  1. Is VM (motor supply) applied and stable?\n"
               "  2. Is VCC (3.3V logic) stable on DRV8434S VCC pin?\n"
               "  3. Check SCK=%d MOSI=%d MISO=%d CS=%d for continuity\n"
               "  4. Verify SPI Mode 1 (CPOL=0, CPHA=1) on oscilloscope\n"
               "  5. On SDO: are STATUS byte bits[7:6] = '11'?\n"
               "     If all 0xFF, SDO is Hi-Z – check VM and nSLEEP pin state\n",
               3,
               DRV8434S_SCK_GPIO, DRV8434S_MOSI_GPIO,
               DRV8434S_MISO_GPIO, DRV8434S_CS_GPIO);
        while (true)
            tight_loop_contents();
    }

    /* ── Verify register defaults against datasheet before config ────── */

    for (uint8_t k = 0; k < s_chain.cfg.n_devices; ++k)
    {
        verify_defaults(k);
    }

    /* ── Configure SPI step/direction mode ────────────────────────────────
     *
     * drv8434s_chain_set_spi_step_mode() performs a read-modify-write on
     * CTRL3 to set SPI_STEP (bit 4) and SPI_DIR (bit 5).  After this,
     * stepping and direction are controlled via the STEP (bit 6) and DIR
     * (bit 7) fields in CTRL3 written by drv8434s_chain_step() and
     * drv8434s_chain_set_dir(). */

    for (uint8_t k = 0; k < s_chain.cfg.n_devices; ++k)
    {
        if (!drv8434s_chain_set_spi_step_mode(&s_chain, k, NULL))
        {
            printf("stepper_test: set SPI step mode failed (dev %u)\n", k);
        }
        else
        {
            /* Verify: read back CTRL3 and confirm SPI_STEP (bit 4) and
             * SPI_DIR (bit 5) are both set.  If either is clear the
             * motion_tick STEP writes will be ignored by the device
             * (hardware STEP/DIR pins would control stepping instead). */
            uint8_t ctrl3 = 0;
            if (drv8434s_chain_read_reg(&s_chain, k, DRV8434S_REG_CTRL3,
                                        &ctrl3, NULL))
            {
                bool spi_step_ok = (ctrl3 & DRV8434S_CTRL3_SPI_STEP) != 0;
                bool spi_dir_ok  = (ctrl3 & DRV8434S_CTRL3_SPI_DIR)  != 0;
                printf("stepper_test: dev %u CTRL3=0x%02X  SPI_STEP=%u SPI_DIR=%u%s\n",
                       k, ctrl3,
                       (unsigned)spi_step_ok, (unsigned)spi_dir_ok,
                       (spi_step_ok && spi_dir_ok) ? "" :
                       "  *** SPI step mode NOT set – steps will be ignored! ***");
                if (!spi_step_ok || !spi_dir_ok)
                {
                    printf("stepper_test: FATAL: SPI_STEP/SPI_DIR not set on dev %u, halting\n", k);
                    while (true) tight_loop_contents();
                }
            }
            else
            {
                printf("stepper_test: dev %u CTRL3 readback failed\n", k);
            }
        }
    }

    /* ── Enable motor outputs (EN_OUT in CTRL2) ──────────────────────── */

    for (uint8_t k = 0; k < s_chain.cfg.n_devices; ++k)
    {
        if (!drv8434s_chain_enable(&s_chain, k, NULL))
        {
            printf("stepper_test: enable outputs failed (dev %u)\n", k);
        }
    }

    /* ── Verify post-configuration register state ─────────────────────── */

    printf("stepper_test: post-config register dump:\n");
    for (uint8_t k = 0; k < s_chain.cfg.n_devices; ++k)
    {
        dump_device_regs(k);
    }

    /* ── Initialise motion engine ─────────────────────────────────────────
     *
     * torque_sample_div = 10 means TRQ_COUNT is sampled every 10th tick
     * to avoid excessive SPI traffic in the repeating timer context.
     * The done callback fires from the tick context when a motion job
     * completes (reached target, torque limit, or fault). */

    if (!drv8434s_motion_init(&s_motion, &s_chain,
                              stepper_motion_done, NULL, 10u))
    {
        printf("stepper_test: motion engine init failed\n");
        return 2;
    }

    printf("stepper_test: chain ready, beginning agitation on device 0\n");

    s_need_new_motion = true;
    s_reverse_next = false;
    s_last_done_ms = to_ms_since_boot(get_absolute_time());

    /* ── Main loop: schedule forward/pause/reverse agitation ──────────── */

    uint32_t s_last_diag_ms = 0u; /* for periodic tick-count print */

    while (true)
    {
        uint32_t now = to_ms_since_boot(get_absolute_time());

        /* ── Periodic diagnostic: confirm timer is firing ──────────────
         * Print every 2 seconds so we can verify the step timer callback
         * is actually executing even when no motion is scheduled. */
        if ((now - s_last_diag_ms) >= 2000u)
        {
            s_last_diag_ms = now;
            uint32_t ticks = s_tick_count;          /* snapshot (volatile) */
            uint32_t faultticks = s_fault_tick_count;
            printf("diag: ticks=%lu  fault_ticks=%lu  timer_active=%u  busy=%u\n",
                   (unsigned long)ticks,
                   (unsigned long)faultticks,
                   (unsigned)s_step_timer_active,
                   (unsigned)drv8434s_motion_is_busy(&s_motion));
        }

        if (s_need_new_motion && !drv8434s_motion_is_busy(&s_motion) &&
            (now - s_last_done_ms) >= 500u)
        {
            s_need_new_motion = false;
            /* Clear any latched faults before each motion attempt so the
             * STATUS byte returned by the first STEP write is clean.
             * A stale fault bit (e.g. OL from power-on) would cause
             * motion_tick to declare MOTION_FAULT on tick 1, stopping
             * the job before any detectable movement occurs. */
            for (uint8_t k = 0; k < s_chain.cfg.n_devices; ++k)
                drv8434s_chain_clear_faults(&s_chain, k, NULL);
            sleep_ms(2); /* allow device to process CLR_FLT internally */

            int32_t steps = (int32_t)AGITATOR_KNEAD_STEPS;
            if (s_reverse_next)
                steps = -steps;

            /* drv8434s_motion_start() queues a non-blocking motion job.
             * target_steps > 0 = forward, < 0 = reverse.
             * torque_limit = 0 disables torque-based stop detection. */
            if (!drv8434s_motion_start(&s_motion, 0, steps, 0))
            {
                printf("stepper_test: motion start failed (dev=0, steps=%ld)\n",
                       (long)steps);
                s_last_done_ms = now; /* back off 500 ms before retry */
            }
            else
            {
                s_tick_count       = 0u; /* reset per-run tick counter */
                s_fault_tick_count = 0u;
                printf("stepper_test: scheduled %s motion %ld steps\n",
                       s_reverse_next ? "reverse" : "forward",
                       (long)steps);
                if (!ensure_step_timer_running(AGITATOR_STEP_DELAY_US))
                {
                    printf("stepper_test: FATAL: add_repeating_timer_us failed – "
                           "no hardware alarm available\n");
                }
                else
                {
                    printf("stepper_test: step timer started (%lu µs/step)\n",
                           (unsigned long)AGITATOR_STEP_DELAY_US);
                }
            }
        }
        tight_loop_contents();
    }
}
