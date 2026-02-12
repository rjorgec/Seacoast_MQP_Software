// ════════════════════════════════════════════════════════════════════════════
//  DRV8434S stepper-motor demo for Raspberry Pi Pico 2 (RP2350, ARM core)
//
//  Features:
//    • SPI initialisation on RP2350
//    • Device probe & register dump
//    • SPI step/direction control mode
//    • Non-blocking motor spin via repeating timer
//    • Software torque limit using TRQ_COUNT register
//    • Fault reporting through nFAULT GPIO interrupt
//
//  Adjust pin defines below to match your wiring.
// ════════════════════════════════════════════════════════════════════════════

#include "drv8434s.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include <stdio.h>
#include <string.h>

// ── Pin definitions (adjust to your board) ──────────────────────────────────

#define SPI_PORT spi0
#define PIN_SCK 18
#define PIN_MOSI 19
#define PIN_MISO 16

#ifndef PIN_CS
#define PIN_CS 13
#endif
#ifndef PIN_RST
#define PIN_RST 12
#endif
#ifndef PIN_NFAULT
#define PIN_NFAULT 11
#endif

// ── Motor configuration ─────────────────────────────────────────────────────

#define MOTOR_MICROSTEP DRV8434S_STEP_1_128 // 1/16 micro-stepping
#define MOTOR_TORQUE 0                      // TRQ DAC (0=100%, 15=25%)
#define MOTOR_DECAY DRV8434S_DECAY_SMART_TUNE_RIPPLE
#define MOTOR_STEP_INTERVAL_US 500 // Step period (µs) → speed
#define MOTOR_STEPS_PER_REV 200    // Full steps per revolution
#define MOTOR_STALL_THRESHOLD 0    // HW stall: TRQ_COUNT floor (12-bit, 0–4095)
#define MOTOR_TORQUE_FLOOR 200     // SW monitor: TRQ_COUNT floor (0–255)
#define TORQUE_CHECK_INTERVAL_MS 1 // How often to poll TRQ_COUNT

// ── Global driver instance ──────────────────────────────────────────────────

static drv8434s_t g_dev;

// ── Stepper state (non-blocking) ────────────────────────────────────────────

typedef struct
{
    bool running;            // motor is spinning
    bool direction;          // false = forward, true = reverse
    uint32_t steps_taken;    // total steps since start
    int32_t target_steps;    // -1 = run forever, else stop after N steps
    repeating_timer_t timer; // Pico SDK repeating timer handle
} stepper_state_t;

static stepper_state_t g_stepper = {0};

// ── Torque monitor state ────────────────────────────────────────────────────

typedef struct
{
    bool enabled;
    uint8_t floor;        // TRQ_COUNT floor (stall when below)
    bool tripped;         // set true when TRQ_COUNT drops below floor
    uint8_t last_reading; // most recent TRQ_COUNT value
    uint32_t blanking;    // callbacks to skip after start (motor spin-up)
    repeating_timer_t timer;
} torque_monitor_t;

static torque_monitor_t g_torque = {0};

// ── Fault state ─────────────────────────────────────────────────────────────

static volatile bool g_fault_latched = false;

// ═════════════════════════════════════════════════════════════════════════════
//  Platform callbacks for drv8434s driver
// ═════════════════════════════════════════════════════════════════════════════

// The DRV8434S requires continuous 16-bit SPI frames (no inter-byte gap).
// Using 8-bit frame size causes the RP2350 SPI hardware to insert a small
// pause between the command byte and data byte, which resets the device's
// internal shift register and produces a loopback echo on SDO.
//
// Solution: configure SPI for 16-bit frames and use spi_write16_read16_blocking
// so all 16 clock cycles are continuous within a single CS assertion.
static int pico_spi_xfer(void *user_ctx, const uint8_t *tx, uint8_t *rx,
                         size_t len)
{
    (void)user_ctx;

    // Standard DRV8434S register transfer: exactly 2 bytes = 16-bit frame
    if (len == 2 && tx && rx)
    {
        uint16_t tx16 = ((uint16_t)tx[0] << 8) | tx[1];
        uint16_t rx16 = 0;
        spi_write16_read16_blocking(SPI_PORT, &tx16, &rx16, 1);
        rx[0] = (uint8_t)(rx16 >> 8);
        rx[1] = (uint8_t)(rx16 & 0xFF);
    }
    else if (len == 2 && tx)
    {
        uint16_t tx16 = ((uint16_t)tx[0] << 8) | tx[1];
        spi_write16_blocking(SPI_PORT, &tx16, 1);
    }
    else if (tx && rx)
    {
        // Fallback for non-standard lengths (e.g. legacy probe_cmd)
        spi_write_read_blocking(SPI_PORT, tx, rx, len);
    }
    else if (tx)
    {
        spi_write_blocking(SPI_PORT, tx, len);
    }
    else if (rx)
    {
        spi_read_blocking(SPI_PORT, 0x00, rx, len);
    }
    else
    {
        return -1;
    }
    return 0;
}

static void pico_cs(void *user_ctx, bool asserted)
{
    (void)user_ctx;
    gpio_put(PIN_CS, asserted ? 0 : 1); // active low
}

static void pico_reset(void *user_ctx, bool asserted)
{
    (void)user_ctx;
    gpio_put(PIN_RST, asserted ? 0 : 1); // active low
}

static void pico_delay_ms(void *user_ctx, unsigned ms)
{
    (void)user_ctx;
    sleep_ms(ms);
}

static void pico_delay_us(void *user_ctx, unsigned us)
{
    (void)user_ctx;
    busy_wait_us_32(us);
}

// ═════════════════════════════════════════════════════════════════════════════
//  Hardware initialisation
// ═════════════════════════════════════════════════════════════════════════════

static void hw_init(void)
{
    stdio_init_all();
    sleep_ms(1000); // allow USB CDC to enumerate

    // ── SPI ──
    // DRV8434S: CPOL=0, CPHA=1 (Mode 1), MSB-first, 16-bit frames.
    // 16-bit frame size prevents inter-byte gap that causes loopback echo.
    spi_init(SPI_PORT, 1000 * 1000); // 1 MHz
    spi_set_format(SPI_PORT, 16, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);

    // ── CS pin ──
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1); // deasserted

    // ── Reset pin ──
    gpio_init(PIN_RST);
    gpio_set_dir(PIN_RST, GPIO_OUT);
    gpio_put(PIN_RST, 1); // not in reset

    // ── nFAULT pin (open-drain, needs pull-up) ──
    gpio_init(PIN_NFAULT);
    gpio_set_dir(PIN_NFAULT, GPIO_IN);
    gpio_pull_up(PIN_NFAULT);
}

// ═════════════════════════════════════════════════════════════════════════════
//  Fault interrupt handler
// ═════════════════════════════════════════════════════════════════════════════

static void nfault_isr(uint gpio, uint32_t events)
{
    if (gpio == PIN_NFAULT && (events & GPIO_IRQ_EDGE_FALL))
    {
        g_fault_latched = true;
        // Stop motor immediately from ISR context
        g_stepper.running = false;
    }
}

static void fault_irq_init(void)
{
    gpio_set_irq_enabled_with_callback(PIN_NFAULT, GPIO_IRQ_EDGE_FALL,
                                       true, nfault_isr);
}

// ═════════════════════════════════════════════════════════════════════════════
//  Register dump (debug helper)
// ═════════════════════════════════════════════════════════════════════════════

static const char *reg_names[DRV8434S_NUM_REGS] = {
    "FAULT", "DIAG1", "DIAG2", "CTRL1", "CTRL2",
    "CTRL3", "CTRL4", "CTRL5", "CTRL6", "CTRL7",
    "CTRL8", "CTRL9"};

static void dump_registers(drv8434s_t *dev)
{
    uint8_t regs[DRV8434S_NUM_REGS];
    if (!drv8434s_read_all_regs(dev, regs))
    {
        printf("  [register dump failed]\n");
        return;
    }
    printf("  ┌─────────┬──────┬──────────┐\n");
    printf("  │ Register │ Addr │   Value  │\n");
    printf("  ├─────────┼──────┼──────────┤\n");
    for (int i = 0; i < DRV8434S_NUM_REGS; ++i)
    {
        printf("  │ %-7s │ 0x%02X │ 0x%02X     │\n",
               reg_names[i], i, regs[i]);
    }
    printf("  └─────────┴──────┴──────────┘\n");
}

static void print_faults(uint8_t faults)
{
    if (faults == 0)
    {
        printf("  No faults.\n");
        return;
    }
    printf("  FAULT register = 0x%02X\n", faults);
    if (faults & DRV8434S_FAULT_FAULT)
        printf("    ├─ FAULT (global)\n");
    if (faults & DRV8434S_FAULT_SPI_ERR)
        printf("    ├─ SPI_ERR\n");
    if (faults & DRV8434S_FAULT_UVLO)
        printf("    ├─ UVLO (under-voltage lockout)\n");
    if (faults & DRV8434S_FAULT_CPUV)
        printf("    ├─ CPUV (charge-pump under-voltage)\n");
    if (faults & DRV8434S_FAULT_OCP)
        printf("    ├─ OCP (over-current)\n");
    if (faults & DRV8434S_FAULT_STL)
        printf("    ├─ STL (stall detected)\n");
    if (faults & DRV8434S_FAULT_TF)
        printf("    ├─ TF (thermal flag)\n");
    if (faults & DRV8434S_FAULT_OL)
        printf("    └─ OL (open load)\n");
}

static void print_fault_info(const drv8434s_fault_info_t *info)
{
    print_faults(info->fault);
    if (info->diag1)
    {
        printf("  DIAG1 (OCP detail) = 0x%02X\n", info->diag1);
        if (info->diag1 & DRV8434S_DIAG1_OCP_LS2_B)
            printf("    ├─ OCP low-side 2 coil B\n");
        if (info->diag1 & DRV8434S_DIAG1_OCP_HS2_B)
            printf("    ├─ OCP high-side 2 coil B\n");
        if (info->diag1 & DRV8434S_DIAG1_OCP_LS1_B)
            printf("    ├─ OCP low-side 1 coil B\n");
        if (info->diag1 & DRV8434S_DIAG1_OCP_HS1_B)
            printf("    ├─ OCP high-side 1 coil B\n");
        if (info->diag1 & DRV8434S_DIAG1_OCP_LS2_A)
            printf("    ├─ OCP low-side 2 coil A\n");
        if (info->diag1 & DRV8434S_DIAG1_OCP_HS2_A)
            printf("    ├─ OCP high-side 2 coil A\n");
        if (info->diag1 & DRV8434S_DIAG1_OCP_LS1_A)
            printf("    ├─ OCP low-side 1 coil A\n");
        if (info->diag1 & DRV8434S_DIAG1_OCP_HS1_A)
            printf("    └─ OCP high-side 1 coil A\n");
    }
    if (info->diag2)
    {
        printf("  DIAG2 (diagnostics) = 0x%02X\n", info->diag2);
        if (info->diag2 & DRV8434S_DIAG2_OTW)
            printf("    ├─ OTW (over-temperature warning)\n");
        if (info->diag2 & DRV8434S_DIAG2_OTS)
            printf("    ├─ OTS (over-temperature shutdown)\n");
        if (info->diag2 & DRV8434S_DIAG2_STL_LRN_OK)
            printf("    ├─ STL_LRN_OK (stall learning complete)\n");
        if (info->diag2 & DRV8434S_DIAG2_STALL)
            printf("    ├─ STALL (stall detected)\n");
        if (info->diag2 & DRV8434S_DIAG2_OL_B)
            printf("    ├─ Open load coil B\n");
        if (info->diag2 & DRV8434S_DIAG2_OL_A)
            printf("    └─ Open load coil A\n");
    }
}

// ═════════════════════════════════════════════════════════════════════════════
//  Non-blocking stepper motor control
// ═════════════════════════════════════════════════════════════════════════════

// Called from repeating timer at MOTOR_STEP_INTERVAL_US
static bool step_timer_callback(repeating_timer_t *rt)
{
    (void)rt;

    if (!g_stepper.running)
    {
        printf("Motor stopped!\n");
        return false; // cancel timer
    }

    // Check torque trip
    if (g_torque.enabled && g_torque.tripped)
    {
        g_stepper.running = false;
        return false;
    }

    // Issue one SPI step
    drv8434s_spi_step(&g_dev);
    g_stepper.steps_taken++;

    // Check if we have a step target
    if (g_stepper.target_steps > 0 &&
        g_stepper.steps_taken >= (uint32_t)g_stepper.target_steps)
    {
        g_stepper.running = false;
        return false; // done
    }

    return true; // keep repeating
}

/// Start the motor spinning (non-blocking).
/// @param direction  false = forward, true = reverse
/// @param steps      Number of steps to take, or -1 to run continuously
/// @param interval_us  Microseconds between steps (controls speed)
/// @return true if motor started successfully
static bool motor_start(bool direction, int32_t steps, int32_t interval_us)
{
    if (g_stepper.running)
    {
        printf("Motor already running.\n");
        return false;
    }

    // Set direction on the device
    drv8434s_set_spi_dir(&g_dev, direction);

    g_stepper.direction = direction;
    g_stepper.steps_taken = 0;
    g_stepper.target_steps = steps;
    g_stepper.running = true;

    // Use negative interval for microsecond precision
    if (!add_repeating_timer_us(-interval_us, step_timer_callback, NULL,
                                &g_stepper.timer))
    {
        g_stepper.running = false;
        printf("Failed to start step timer.\n");
        return false;
    }

    printf("Motor started: dir=%s, steps=%ld, interval=%ld us\n",
           direction ? "REV" : "FWD",
           (long)steps, (long)interval_us);
    return true;
}

/// Stop the motor.
static void motor_stop(void)
{
    if (g_stepper.running)
    {
        g_stepper.running = false;
        cancel_repeating_timer(&g_stepper.timer);
    }
    printf("Motor stopped after %lu steps.\n",
           (unsigned long)g_stepper.steps_taken);
}

// ═════════════════════════════════════════════════════════════════════════════
//  Software torque-limit monitor (non-blocking)
// ═════════════════════════════════════════════════════════════════════════════

// Called periodically to read TRQ_COUNT and check against floor
static bool torque_monitor_callback(repeating_timer_t *rt)
{
    (void)rt;
    if (!g_torque.enabled)
        return false;

    // Blanking period: skip checks while motor spins up
    if (g_torque.blanking > 0)
    {
        g_torque.blanking--;
        return true;
    }

    // Only check when motor is actually running
    if (!g_stepper.running)
        return true;

    uint8_t trq = 0;
    if (drv8434s_read_torque_count(&g_dev, &trq))
    {
        g_torque.last_reading = trq;
        if (trq < g_torque.floor)
        {
            g_torque.tripped = true;
            // The step timer callback will see this flag and stop
            printf("*** STALL WARNING: TRQ_COUNT=%u < floor=%u ***\n",
                   trq, g_torque.floor);
        }
    }
    return true; // keep monitoring
}

/// Start monitoring TRQ_COUNT with a software stall floor.
/// If TRQ_COUNT drops below `floor`, the motor will be stopped.
/// A blanking period of 500 ms lets the motor reach steady-state first.
static bool torque_monitor_start(uint8_t floor, uint32_t check_ms)
{
    if (g_torque.enabled)
    {
        printf("Torque monitor already running.\n");
        return false;
    }

    g_torque.enabled = true;
    g_torque.floor = floor;
    g_torque.tripped = false;
    g_torque.last_reading = 0;
    g_torque.blanking = 500 / check_ms; // ~500 ms blanking period

    if (!add_repeating_timer_ms((int32_t)check_ms, torque_monitor_callback,
                                NULL, &g_torque.timer))
    {
        g_torque.enabled = false;
        printf("Failed to start torque monitor timer.\n");
        return false;
    }

    printf("Torque monitor started: floor=%u, check every %lu ms\n",
           floor, (unsigned long)check_ms);
    return true;
}

/// Stop the torque monitor.
static void torque_monitor_stop(void)
{
    if (g_torque.enabled)
    {
        g_torque.enabled = false;
        cancel_repeating_timer(&g_torque.timer);
    }
}

// ═════════════════════════════════════════════════════════════════════════════
//  Fault recovery (application-level)
// ═════════════════════════════════════════════════════════════════════════════

// Maximum consecutive recovery attempts before we give up and halt the motor.
#define MAX_FAULT_RETRIES 3

static uint32_t g_fault_recover_count = 0; // lifetime recovery counter

/// Full fault-recovery sequence:
///  1. Stop motor & torque monitor
///  2. Read and log full diagnostics (FAULT + DIAG1 + DIAG2)
///  3. Call drv8434s_recover_from_fault() (CLR_FLT → hardware reset)
///  4. Re-apply motor configuration
///  5. Restart motor & torque monitor
///
/// @return true if the driver is fault-free and the motor has been restarted
static bool motor_recover_from_fault(void)
{
    // ── 1. Stop everything ──────────────────────────────────────────────
    motor_stop();
    torque_monitor_stop();

    printf("  [recovery] Attempting fault recovery (#%lu)...\n",
           (unsigned long)++g_fault_recover_count);

    // ── 2–3. Driver-level recovery (CLR_FLT then hardware reset) ────────
    drv8434s_fault_info_t info;
    drv8434s_recover_result_t result =
        drv8434s_recover_from_fault(&g_dev, &info, 10);

    // Log pre-recovery diagnostics that were captured by the driver
    printf("  [recovery] Pre-recovery diagnostics:\n");
    print_fault_info(&info);

    switch (result)
    {
    case DRV8434S_RECOVER_OK:
        printf("  [recovery] No fault was active (spurious nFAULT edge).\n");
        break;
    case DRV8434S_RECOVER_CLR_OK:
        printf("  [recovery] Faults cleared via CLR_FLT.\n");
        break;
    case DRV8434S_RECOVER_RESET_OK:
        printf("  [recovery] Faults cleared after hardware reset.\n");
        printf("  [recovery] Registers restored from shadow cache.\n");
        break;
    case DRV8434S_RECOVER_STILL_FAULTED:
        printf("  [recovery] ** Faults persist after all attempts! **\n");
        printf("  [recovery] Post-recovery state:\n");
        print_fault_info(&info);
        return false;
    case DRV8434S_RECOVER_SPI_ERROR:
        printf("  [recovery] ** SPI communication failure! **\n");
        return false;
    }

    // ── 4. Re-apply motor configuration ─────────────────────────────────
    //
    // After a CLR_FLT the registers are still intact.  After a hardware
    // reset the driver-level function already restored them from the shadow
    // cache, but EN_OUT was intentionally left off — we re-enable here.
    drv8434s_set_spi_step_mode(&g_dev);
    drv8434s_set_microstep(&g_dev, MOTOR_MICROSTEP);
    drv8434s_set_torque(&g_dev, MOTOR_TORQUE);
    drv8434s_set_decay(&g_dev, MOTOR_DECAY);
    drv8434s_enable_stall_detection(&g_dev);
    drv8434s_enable(&g_dev);
    printf("  [recovery] Motor re-configured, stall detection + outputs enabled.\n");

    // Brief settling delay before restarting motion
    sleep_ms(50);

    // ── 5. Restart motor & torque monitor ───────────────────────────────
    torque_monitor_start(MOTOR_TORQUE_FLOOR, TORQUE_CHECK_INTERVAL_MS);
    motor_start(g_stepper.direction, -1, MOTOR_STEP_INTERVAL_US);
    printf("  [recovery] Motor restarted.\n");

    return true;
}

// ═════════════════════════════════════════════════════════════════════════════
//  Main application
// ═════════════════════════════════════════════════════════════════════════════

int main(void)
{
    hw_init();
    sleep_ms(5000);
    printf("\n╔══════════════════════════════════════════╗\n");
    printf("║  DRV8434S Stepper Demo (RP2350)          ║\n");
    printf("╚══════════════════════════════════════════╝\n\n");

    // ── Initialise driver context ───────────────────────────────────────
    drv8434s_config_t cfg = {
        .user_ctx = NULL,
        .spi_xfer = pico_spi_xfer,
        .cs = pico_cs,
        .reset = pico_reset,
        .delay_ms = pico_delay_ms,
        .delay_us = pico_delay_us,
    };

    if (!drv8434s_init(&g_dev, &cfg))
    {
        printf("drv8434s_init FAILED\n");
        while (1)
            tight_loop_contents();
    }
    printf("[init] OK\n");

    // ── Reset & probe ───────────────────────────────────────────────────
    drv8434s_reset(&g_dev, 10);

    bool present = drv8434s_probe(&g_dev);
    printf("[probe] %s\n", present ? "device detected" : "NOT detected");
    if (!present)
    {
        printf("Halting — no device on SPI bus.\n");
        while (1)
            tight_loop_contents();
    }

    // ── Register dump (initial state) ───────────────────────────────────
    printf("\nRegisters after reset:\n");
    dump_registers(&g_dev);

    // ── Read & report faults ────────────────────────────────────────────
    uint8_t faults;
    drv8434s_read_fault(&g_dev, &faults);
    printf("\nFault status:\n");
    print_faults(faults);

    if (faults)
    {
        printf("Clearing faults...\n");
        drv8434s_clear_faults(&g_dev);
        sleep_ms(5);
        drv8434s_read_fault(&g_dev, &faults);
        printf("After clear: ");
        print_faults(faults);
    }

    // ── Configure motor ─────────────────────────────────────────────────
    printf("\nConfiguring motor:\n");

    // Enable SPI step/direction control (STEP/DIR via SPI instead of HW pins)
    drv8434s_set_spi_step_mode(&g_dev);
    printf("  SPI step/dir mode enabled\n");

    // Microstep resolution
    drv8434s_set_microstep(&g_dev, MOTOR_MICROSTEP);
    printf("  Microstep: 1/%d\n", 1 << MOTOR_MICROSTEP);

    // Torque (current scale: 0=100%, 15=25% per Table 7-6)
    drv8434s_set_torque(&g_dev, MOTOR_TORQUE);
    printf("  Torque DAC: %d (100%% - %d × 6.25%% = %.1f%%)\n",
           MOTOR_TORQUE, MOTOR_TORQUE,
           100.0 - MOTOR_TORQUE * 6.25);

    // Decay mode
    drv8434s_set_decay(&g_dev, MOTOR_DECAY);
    printf("  Decay: smart-tune ripple\n");

    // Enable outputs
    drv8434s_enable(&g_dev);
    printf("  Outputs ENABLED\n");

    // ── Register dump (after configuration) ─────────────────────────────
    printf("\nRegisters after configuration:\n");
    dump_registers(&g_dev);

    // ── Stall learning ──────────────────────────────────────────────────
    // The device briefly stalls the motor to learn the ideal STALL_TH.
    // After learning, hardware stall detection is enabled so nFAULT fires
    // when TRQ_COUNT drops below the learned threshold.
    printf("\n── Stall learning ──\n");
    drv8434s_enable_stall_detection(&g_dev);
    drv8434s_start_stall_learning(&g_dev);
    printf("  Learning started, waiting for STL_LRN_OK...\n");

    for (int i = 0; i < 50; ++i) // timeout ~5 s
    {
        sleep_ms(100);
        if (drv8434s_is_stall_learned(&g_dev))
        {
            printf("  Stall learning complete!\n");

            // Read back the learned STALL_TH for display
            uint8_t stall_lo = 0, stall_hi_reg = 0;
            drv8434s_read_reg(&g_dev, DRV8434S_REG_CTRL6, &stall_lo);
            drv8434s_read_reg(&g_dev, DRV8434S_REG_CTRL7, &stall_hi_reg);
            uint16_t learned_th = stall_lo |
                                  ((uint16_t)(stall_hi_reg & DRV8434S_CTRL7_STALL_TH_HI_MASK) << 8);
            printf("  Learned STALL_TH = %u\n", learned_th);
            break;
        }
    }
    if (!drv8434s_is_stall_learned(&g_dev))
    {
        printf("  ** Stall learning timed out — using default STALL_TH **\n");
    }

    // Clear any faults that may have been generated during learning
    drv8434s_clear_faults(&g_dev);
    sleep_ms(5);

    // ── Enable fault interrupt ──────────────────────────────────────────
    fault_irq_init();
    printf("\nnFAULT interrupt enabled on GPIO %d\n", PIN_NFAULT);

    // ── Start torque monitor ────────────────────────────────────────────
    torque_monitor_start(MOTOR_TORQUE_FLOOR, TORQUE_CHECK_INTERVAL_MS);

    // ── Spin motor (non-blocking, continuous) ───────────────────────────
    printf("\n── Starting motor ──\n");
    motor_start(false, -1, MOTOR_STEP_INTERVAL_US);

    // ── Main loop: periodic status reports ──────────────────────────────
    uint32_t report_count = 0;
    while (true)
    {
        sleep_ms(2000);
        report_count++;

        printf("\n── Status report #%lu ──\n", (unsigned long)report_count);

        // Check for hardware faults (latched by ISR)
        if (g_fault_latched)
        {
            g_fault_latched = false;
            printf("  ** HARDWARE FAULT DETECTED **\n");

            if (!motor_recover_from_fault())
            {
                printf("  ** Recovery FAILED — motor halted. **\n");
                printf("  Will retry on next cycle...\n");
            }
            continue; // skip normal status for this cycle
        }

        // Check torque monitor
        if (g_torque.tripped)
        {
            printf("  ** Torque limit was exceeded (TRQ_COUNT=%u) **\n",
                   g_torque.last_reading);
            printf("  Reducing torque and restarting...\n");

            // Recovery: reduce torque, clear trip, restart motor
            motor_stop();
            torque_monitor_stop();

            uint8_t new_trq = MOTOR_TORQUE > 2 ? MOTOR_TORQUE - 2 : 1;
            drv8434s_set_torque(&g_dev, new_trq);
            printf("  New torque DAC: %u/15\n", new_trq);

            sleep_ms(100);
            torque_monitor_start(MOTOR_TORQUE_FLOOR,
                                 TORQUE_CHECK_INTERVAL_MS);
            motor_start(false, -1, MOTOR_STEP_INTERVAL_US);
            continue;
        }

        // Print current status
        printf("  Motor: %s, steps=%lu, dir=%s\n",
               g_stepper.running ? "RUNNING" : "STOPPED",
               (unsigned long)g_stepper.steps_taken,
               g_stepper.direction ? "REV" : "FWD");
        printf("  TRQ_COUNT: %u (floor=%u)\n",
               g_torque.last_reading, g_torque.floor);

        // Periodically reverse direction (every 10 reports ≈ 20 s)
        if (report_count % 10 == 0 && g_stepper.running)
        {
            bool new_dir = !g_stepper.direction;
            motor_stop();
            sleep_ms(200);
            motor_start(new_dir, -1, MOTOR_STEP_INTERVAL_US);
            printf("  Direction reversed → %s\n", new_dir ? "REV" : "FWD");
        }

        // Periodic register dump (every 5 reports ≈ 10 s)
        if (report_count % 5 == 0)
        {
            printf("  Full register state:\n");
            dump_registers(&g_dev);
        }
    }

    return 0;
}
