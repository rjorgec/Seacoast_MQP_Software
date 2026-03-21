#pragma once

// Central place for *all* Pico-side pin assignments.
// Keep this file platform-agnostic: no Pico SDK headers required.

// =========================
// UART link to ESP
// =========================
// Choose uart0 or uart1 later; for now just use integers and map in code.
// In code you can do: uart_inst_t *u = (PICO_UART_ID==0)? uart0 : uart1;

#ifndef PICO_UART_ID
#define PICO_UART_ID 0 // 0 = uart0, 1 = uart1
#endif

#ifndef PICO_UART_BAUD
#define PICO_UART_BAUD 115200
#endif

#ifndef PICO_UART_TX_GPIO
#define PICO_UART_TX_GPIO 0 // set later (e.g. 4)
#endif

#ifndef PICO_UART_RX_GPIO
#define PICO_UART_RX_GPIO 1 // set later (e.g. 5)
#endif

// =========================
// DRV8263 motor driver pins
// =========================
// Required mapping decisions for real hardware bring-up:
// - DRV8263_CTRL_A_GPIO: PWM-capable pin driving IN1
// - DRV8263_CTRL_B_GPIO: PWM-capable pin driving IN2
// - DRV8263_SENSE_GPIO: ADC-capable pin connected to current sense
// - DRV8263_SENSE_ADC_CH: ADC channel index for DRV8263_SENSE_GPIO

#ifndef DRV8263_CTRL_A_GPIO
#define DRV8263_CTRL_A_GPIO 6 // PWM capable GPIO
#endif

#ifndef DRV8263_CTRL_B_GPIO
#define DRV8263_CTRL_B_GPIO 7 // PWM capable GPIO
#endif

#ifndef DRV8263_SENSE_GPIO
#define DRV8263_SENSE_GPIO                                                     \
  28 // ADC-capable GPIO (commonly 26/27/28 on RP2040-style)
#endif

// If you use RP2040-style ADC GPIO mapping, channel = gpio - 26.
// You can override this if needed.
#ifndef DRV8263_SENSE_ADC_CH
#define DRV8263_SENSE_ADC_CH ((DRV8263_SENSE_GPIO) - 26)
#endif

#ifndef DRV8263_DEFAULT_PWM_HZ
#define DRV8263_DEFAULT_PWM_HZ 20000
#endif

#ifndef DRV8263_DEFAULT_LOW_TH
#define DRV8263_DEFAULT_LOW_TH 30
#endif

#ifndef DRV8263_DEFAULT_HIGH_TH
#define DRV8263_DEFAULT_HIGH_TH 100
#endif

#ifndef DRV8263_DEFAULT_STARTUP_BLANKING_MS
#define DRV8263_DEFAULT_STARTUP_BLANKING_MS 250
#endif

#ifndef DRV8263_DEFAULT_CHECK_INTERVAL_MS
#define DRV8263_DEFAULT_CHECK_INTERVAL_MS 5
#endif

// =========================
// HX711 scale pins
// =========================
// Required mapping decisions for real hardware bring-up:
// - HX711_CLK_GPIO: Clock pin
// - HX711_DATA_GPIO: Data pin

#ifndef HX711_CLK_GPIO
#define HX711_CLK_GPIO 14
#endif

#ifndef HX711_DATA_GPIO
#define HX711_DATA_GPIO 15
#endif

#ifndef HX711_DEFAULT_RATE_US
#define HX711_DEFAULT_RATE_US 250000
#endif

// =========================
// DRV8434S stepper SPI
// =========================
// All DRV8434S devices in the daisy chain share the same MOSI/MISO/SCK and
// a single CS line.  Adjust pin numbers and DRV8434S_N_DEVICES to match your
// PCB layout.  Set DRV8434S_CS_GPIO to -1 (and SPI pins to -1) to disable
// the stepper subsystem at compile time.

// ********PIN 3 cannot drive low!! Blown up!! Must use pin 10 instead********

#ifndef DRV8434S_SPI_ID
#define DRV8434S_SPI_ID 1 // 0 = spi0, 1 = spi1
#endif

#ifndef DRV8434S_SPI_BAUD
#define DRV8434S_SPI_BAUD 1000000 // 1 MHz (device supports up to 10 MHz)
#endif

#ifndef DRV8434S_SCK_GPIO
#define DRV8434S_SCK_GPIO 10
#endif

#ifndef DRV8434S_MOSI_GPIO
#define DRV8434S_MOSI_GPIO 11
#endif

#ifndef DRV8434S_MISO_GPIO
#define DRV8434S_MISO_GPIO 12
#endif

// Single shared chip-select for the entire daisy chain.
#ifndef DRV8434S_CS_GPIO
#define DRV8434S_CS_GPIO 13
#endif

// Number of DRV8434S devices wired in series.B
// 1 = single device (still uses the daisy-chain framing, which is valid).
// Set to 0 to disable the stepper subsystem entirely.
#ifndef DRV8434S_N_DEVICES
#define DRV8434S_N_DEVICES 3
#endif

// Interval between periodic idle SPI health checks (milliseconds).
// When no motion is active, the FAULT register of each device is polled
// at this rate.  A read failure logs a warning and attempts fault recovery.
#ifndef DRV8434S_SPI_WATCHDOG_INTERVAL_MS
#define DRV8434S_SPI_WATCHDOG_INTERVAL_MS 2000
#endif

// =========================
// Optional compile-time guards
// =========================
// Turn these on once you're ready to require real pin selection.
#ifdef REQUIRE_REAL_PINS
#if (PICO_UART_TX_GPIO < 0) || (PICO_UART_RX_GPIO < 0)
#error                                                                         \
    "Set PICO_UART_TX_GPIO and PICO_UART_RX_GPIO in board_pins.h (or via -D defines)."
#endif

#if (DRV8263_CTRL_A_GPIO < 0) || (DRV8263_CTRL_B_GPIO < 0) ||                  \
    (DRV8263_SENSE_GPIO < 0)
#error                                                                         \
    "Set DRV8263_CTRL_A_GPIO / DRV8263_CTRL_B_GPIO / DRV8263_SENSE_GPIO in board_pins.h."
#endif

#if (DRV8263_SENSE_ADC_CH < 0) || (DRV8263_SENSE_ADC_CH > 3)
#error "DRV8263_SENSE_ADC_CH out of range (expected 0..3). Check ADC mapping."
#endif
#endif

/* ================================================================== */
/*  Stepper position constants (DRV8434S, steps from home)             */
/* ================================================================== */

/* Arm stepper (device 0).
 * All arm positions are measured from the physical home hard-stop found by
 * MSG_ARM_HOME. Successful homing then backs off by ARM_HOME_BACKOFF_STEPS, so
 * the idle post-home position is typically negative. */
#ifndef ARM_STEPS_PRESS
#define ARM_STEPS_PRESS -3500 /* steps to press attachment */
#endif
#ifndef ARM_STEPS_POS1
#define ARM_STEPS_POS1 -500 /* absolute position 1 */
#endif
#ifndef ARM_STEPS_POS2
#define ARM_STEPS_POS2 -100 /* absolute position 2 */
#endif
#ifndef ARM_PRESS_STALL_WINDOW_STEPS
#define ARM_PRESS_STALL_WINDOW_STEPS 50 /* steps to confirm press stall */
#endif
#ifndef ARM_PRESS_RETRY_MAX_RETRIES
#define ARM_PRESS_RETRY_MAX_RETRIES 5u /* extra press attempts if vacuum RPM does not change */
#endif
#ifndef ARM_PRESS_RETRY_BACKOFF_STEPS
#define ARM_PRESS_RETRY_BACKOFF_STEPS 150 /* release distance before a retry press */
#endif
#ifndef ARM_PRESS_RETRY_VERIFY_TIMEOUT_MS
#define ARM_PRESS_RETRY_VERIFY_TIMEOUT_MS 300u /* wait this long for an RPM response after press */
#endif
#ifndef ARM_PRESS_RETRY_RPM_DELTA
#define ARM_PRESS_RETRY_RPM_DELTA 2000u /* minimum absolute RPM change that counts as a successful press */
#endif
#ifndef ARM_MOTION_TIMEOUT_MS
#define ARM_MOTION_TIMEOUT_MS 5000
#endif
#ifndef ARM_HOME_SEARCH_STEPS
#define ARM_HOME_SEARCH_STEPS 5000
#endif
#ifndef ARM_HOME_STEP_DELAY_US
#define ARM_HOME_STEP_DELAY_US 4000u //arm speed for home
#endif
#ifndef ARM_HOME_TIMEOUT_MS
#define ARM_HOME_TIMEOUT_MS 15000
#endif
#ifndef ARM_HOME_TORQUE_LIMIT
#define ARM_HOME_TORQUE_LIMIT 120u  //
#endif
#ifndef ARM_HOME_TORQUE_BLANK_STEPS
#define ARM_HOME_TORQUE_BLANK_STEPS 100u
#endif
#ifndef ARM_HOME_TORQUE_SAMPLE_DIV
#define ARM_HOME_TORQUE_SAMPLE_DIV 1u
#endif
#ifndef ARM_HOME_BACKOFF_STEPS
#define ARM_HOME_BACKOFF_STEPS 100
#endif

/* Rack stepper (device 1) */
#ifndef RACK_STEPS_EXTEND
#define RACK_STEPS_EXTEND 800
#endif
#ifndef RACK_STEPS_PRESS
#define RACK_STEPS_PRESS 1200
#endif
#ifndef RACK_MOTION_TIMEOUT_MS
#define RACK_MOTION_TIMEOUT_MS 5000
#endif

/* Turntable stepper (device 2) */
#ifndef TURNTABLE_STEPS_A
#define TURNTABLE_STEPS_A 0
#endif
#ifndef TURNTABLE_STEPS_B
#define TURNTABLE_STEPS_B 500
#endif
#ifndef TURNTABLE_STEPS_C
#define TURNTABLE_STEPS_C 1000
#endif
#ifndef TURNTABLE_STEPS_D
#define TURNTABLE_STEPS_D 1500
#endif
#ifndef TURNTABLE_MOTION_TIMEOUT_MS
#define TURNTABLE_MOTION_TIMEOUT_MS 8000
#endif

/* ================================================================== */
/*  Flap (DRV8263 primary instance) thresholds                         */
/* ================================================================== */

#ifndef FLAP_OPEN_SPEED_PWM
#define FLAP_OPEN_SPEED_PWM 2500 /* slow speed open */
#endif
#ifndef FLAP_OPEN_CURRENT_DROP_TH
#define FLAP_OPEN_CURRENT_DROP_TH 50 /* ADC counts -- open-circuit endpoint */
#endif
#ifndef FLAP_CLOSE_SPEED_PWM
#define FLAP_CLOSE_SPEED_PWM 3000 /* reduced speed on close */
#endif
#ifndef FLAP_CLOSE_TORQUE_TH
#define FLAP_CLOSE_TORQUE_TH 300 /* ADC counts -- stall/torque threshold */
#endif
#ifndef FLAP_MONITOR_INTERVAL_MS
#define FLAP_MONITOR_INTERVAL_MS 20
#endif
#ifndef FLAP_MOTION_TIMEOUT_MS
#define FLAP_MOTION_TIMEOUT_MS 15000
#endif

/* ================================================================== */
/*  Second flap linear actuator (DRV8263 instance 2)                   */
/* ================================================================== */
#ifndef FLAP2_CTRL_A_PIN
#define FLAP2_CTRL_A_PIN 8 /* GP8 */
#endif
#ifndef FLAP2_CTRL_B_PIN
#define FLAP2_CTRL_B_PIN 9 /* GP9 */
#endif
#ifndef FLAP2_ADC_SENSE_PIN
#define FLAP2_ADC_SENSE_PIN 27 /* GP27 — ADC channel 1 */
#endif
#ifndef FLAP2_ADC_CHANNEL
#define FLAP2_ADC_CHANNEL 1
#endif
/* Flap2 shares the same threshold constants as Flap1:
 * FLAP_OPEN_SPEED_PWM, FLAP_OPEN_CURRENT_DROP_TH,
 * FLAP_CLOSE_SPEED_PWM, FLAP_CLOSE_TORQUE_TH,
 * FLAP_MONITOR_INTERVAL_MS, FLAP_MOTION_TIMEOUT_MS
 */

/* ================================================================== */
/*  Hot wire + Vacuum Pump 2 (DRV8263 independent H-bridge instance)  */
/*                                                                     */
/*  The DRV8263 is configured in independent half-bridge mode:         */
/*    IN1 (HOTWIRE_PIN_IN1) drives the nichrome hot wire.              */
/*    IN2 (HOTWIRE_PIN_IN2) drives vacuum pump 2.                      */
/*  Both outputs are fully independent and can run simultaneously.     */
/* ================================================================== */

#ifndef HOTWIRE_PIN_IN1
#define HOTWIRE_PIN_IN1 4 /* GP4 -- drives hot wire (IN1) */
#endif
#ifndef HOTWIRE_PIN_IN2
#define HOTWIRE_PIN_IN2 5 /* GP5 -- drives vacuum pump 2 (IN2) */
#endif

/* ADC sense pin: unused in independent H-bridge mode.
 * Current regulation for the hot wire is set by an external Rsense resistor
 * and is handled internally by the DRV8263 — software does not monitor current.
 * The pin is initialised by drv8263_init() but readings are not used for
 * control. */
#ifndef HOTWIRE_ADC_SENSE_PIN
#define HOTWIRE_ADC_SENSE_PIN                                                  \
  26 /* GP26 -- ADC channel 0 (sense unused, see above) */
#endif
#ifndef HOTWIRE_ADC_CHANNEL
#define HOTWIRE_ADC_CHANNEL 0
#endif

/* Full on (4095 = 12-bit max).  Current is set by external Rsense on the
 * DRV8263; no PWM duty cycle tuning is required in software. */
#ifndef HOTWIRE_ENABLE_DUTY
#define HOTWIRE_ENABLE_DUTY 4095
#endif
#ifndef HOTWIRE_MONITOR_INTERVAL_MS
#define HOTWIRE_MONITOR_INTERVAL_MS 50
#endif

/* ================================================================== */
/*  Vacuum pump                                                         */
/* ================================================================== */

#ifndef VACUUM_TRIGGER_PIN
#define VACUUM_TRIGGER_PIN 2 /* GP2 -- digital output */
#endif
#ifndef VACUUM_RPM_SENSE_PIN
#define VACUUM_RPM_SENSE_PIN                                                   \
  3 /* GP3 -- rising-edge interrupt input (let's hope input still works on     \
       this pin) */
#endif
#ifndef VACUUM_PULSES_PER_REV
#define VACUUM_PULSES_PER_REV 1 /* adjust to match pump sensor */
#endif
#ifndef VACUUM_RPM_SAMPLE_MS
#define VACUUM_RPM_SAMPLE_MS 100 /* RPM sampling window */
#endif
#ifndef VACUUM_RPM_BLOCKED_THRESHOLD
#define VACUUM_RPM_BLOCKED_THRESHOLD 400 /* RPM below this = blocked */
#endif
#ifndef VACUUM_STATUS_SEND_INTERVAL_MS
#define VACUUM_STATUS_SEND_INTERVAL_MS 5000 /* unsolicited status period */
#endif

/* ================================================================== */
/*  DRV8434S device index assignments within the daisy chain           */
/*  Increment DRV8434S_N_DEVICES when adding new devices.             */
/* ================================================================== */

#ifndef STEPPER_DEV_ROT_ARM
#define STEPPER_DEV_ROT_ARM 0 /* Rotary suction arm (MSG_ARM_MOVE) */
#endif
// #ifndef STEPPER_DEV_LIN_ARM
// #define STEPPER_DEV_LIN_ARM 1 /* Linear vacuum arm (MSG_RACK_MOVE) */
// #endif
#ifndef STEPPER_DEV_TURNTABLE
#define STEPPER_DEV_TURNTABLE 1 /* Turntable / platform */
#endif
/* Uncomment each entry and bump DRV8434S_N_DEVICES when wired: */
#ifndef STEPPER_DEV_AGITATOR
#define STEPPER_DEV_AGITATOR 2 /* Agitator eccentric arm */
#endif
// #define STEPPER_DEV_HW_CARRIAGE 4  /* Hot wire carriage traverse */
// #define STEPPER_DEV_INDEXER     5  /* Bag depth/eject rack (indexer) */

/* Default step delay used by the high-level stepper handlers (µs per step). */
#ifndef STEPPER_DEFAULT_STEP_DELAY_US
#define STEPPER_DEFAULT_STEP_DELAY_US 1000u
#endif
#ifndef STEPPER_DEFAULT_TORQUE_SAMPLE_DIV
#define STEPPER_DEFAULT_TORQUE_SAMPLE_DIV 10u
#endif

/* ================================================================== */
/*  HX711 load cell calibration                                        */
/* ================================================================== */

/* Reference unit (slope) — calibrate with known weight. */
#ifndef HX711_REF_UNIT
#define HX711_REF_UNIT (-165)
#endif
/* Zero offset — calibrate by taring with empty platform. */
#ifndef HX711_ZERO_OFFSET
#define HX711_ZERO_OFFSET 8130430
#endif

/* ================================================================== */
/*  Spawn dosing control loop (MSG_DISPENSE_SPAWN / Pico-side)         */
/*                                                                      */
/*  All tunables centralised here.  Override via -D compiler flags or  */
/*  by editing this block before calibration.                           */
/* ================================================================== */

/* ── Timing ────────────────────────────────────────────────────────── */

#ifndef SPAWN_TIMER_PERIOD_MS
#define SPAWN_TIMER_PERIOD_MS 50u /* control loop period (ms); range 20–100 */
#endif
#ifndef SPAWN_FLOW_WINDOW_MS
#define SPAWN_FLOW_WINDOW_MS 100u /* flow-rate measurement window (ms) */
#endif
#ifndef SPAWN_SCALE_READ_SAMPLES
#define SPAWN_SCALE_READ_SAMPLES 1u /* HX711 averaging samples per tick */
#endif

/* ── Flow detection / agitation ───────────────────────────────────── */

#ifndef SPAWN_FLOW_NOFLOW_UG
#define SPAWN_FLOW_NOFLOW_UG 500000u /* min µg/window to count as flowing */
#endif
#ifndef SPAWN_FLOW_MIN_UG
#define SPAWN_FLOW_MIN_UG 1000000u /* low-end flow target (µg/window) */
#endif
#ifndef SPAWN_FLOW_MAX_UG
#define SPAWN_FLOW_MAX_UG 5000000u /* high-end flow target (µg/window) */
#endif
#ifndef SPAWN_MAX_RETRIES
#define SPAWN_MAX_RETRIES 100 /* agitation retries before bag-empty */
#endif
#ifndef SPAWN_AGITATE_MS
#define SPAWN_AGITATE_MS 2000u /* agitation hold-off duration (ms) */
#endif
#ifndef SPAWN_STARTUP_FLOW_DETECT_UG
#define SPAWN_STARTUP_FLOW_DETECT_UG                                           \
  1000000u /* min µg change to confirm prime flow */
#endif

/* ── EMA filtering ─────────────────────────────────────────────────── *
 * Alpha scaled by 1000 (integer maths, no floating point in ISR).     *
 * alpha_x1000 = 250 ≈ α = 0.25 (range 50–500).                        *
 * Higher = faster response; lower = more noise rejection.              */

#ifndef SPAWN_EMA_ALPHA_X1000
#define SPAWN_EMA_ALPHA_X1000                                                  \
  250u /* EMA smoothing factor ×1000; range 50–500 */
#endif

/* ── Flow spike clamp ──────────────────────────────────────────────── *
 * Any per-tick delta above this value is treated as sensor noise and   *
 * clamped.  At 80 SPS, 50 ms window ≈ 4 samples.  A realistic 20 g/s *
 * flow yields ~1 000 000 µg/tick at 50 ms period; 10 000 000 µg/tick  *
 * is an implausible spike.                                             */

#ifndef SPAWN_FLOW_SPIKE_CLAMP_UG
#define SPAWN_FLOW_SPIKE_CLAMP_UG 100000000u /* max believable µg per tick */
#endif

/* ── Nudge / rate limiting ─────────────────────────────────────────── */

#ifndef SPAWN_TICK_DEADBAND
#define SPAWN_TICK_DEADBAND 50000u /* ±µg/tick deadband — no nudge needed */
#endif
#ifndef SPAWN_NUDGE_OPEN_MS
#define SPAWN_NUDGE_OPEN_MS 300u /* ms to nudge flap open per control tick */
#endif
#ifndef SPAWN_NUDGE_CLOSE_MS
#define SPAWN_NUDGE_CLOSE_MS 200u /* ms to nudge flap closed per control tick  \
                                   */
#endif
#ifndef SPAWN_MAX_OPEN_NUDGES
#define SPAWN_MAX_OPEN_NUDGES                                                  \
  8u /* max consecutive open nudges with no flow increase */
#endif
#ifndef SPAWN_DIRECTION_REVERSAL_HOLDOFF_MS
#define SPAWN_DIRECTION_REVERSAL_HOLDOFF_MS                                    \
  350u /* minimum time before allowing an immediate nudge direction reversal */
#endif

/* ── Proportional gain thresholds ──────────────────────────────────── */

#ifndef SPAWN_PROP_UPPER
#define SPAWN_PROP_UPPER 2u /* remaining > target/UPPER → max flow rate */
#endif
#ifndef SPAWN_PROP_LOWER
#define SPAWN_PROP_LOWER 10u /* remaining < target/LOWER → min flow rate */
#endif

/* ── Homing (optional re-zero to closed endpoint before dose) ──────── */

#ifndef SPAWN_HOME_TIMEOUT_MS
#define SPAWN_HOME_TIMEOUT_MS                                                  \
  6000u /* max time to home flaps (ms); ~2× full stroke */
#endif

/* ── Close confirmation ─────────────────────────────────────────────── *
 * After issuing any close command, wait up to this long for the flap   *
 * SM to reach FLAP_SM_CLOSED before declaring a timeout and proceeding *
 * anyway.                                                               */

#ifndef SPAWN_CLOSE_CONFIRM_TIMEOUT_MS
#define SPAWN_CLOSE_CONFIRM_TIMEOUT_MS                                         \
  6000u /* ms to wait for close endpoint detect */
#endif

/* ── Fast-close PWM ─────────────────────────────────────────────────── *
 * Used at end-of-dose and in Finish A close-early step.  Higher than   *
 * the normal close PWM to minimise the coast distance after the        *
 * command is issued.                                                   */

#ifndef SPAWN_FAST_CLOSE_PWM
#define SPAWN_FAST_CLOSE_PWM                                                   \
  4095u /* max PWM for fast close (safe if torque monitoring active) */
#endif

/* ── Estimated close latency (for Finish A overshoot prediction) ────── *
 * Expected time from "issue close" to flaps actually seating (ms).     *
 * Used in the overshoot risk calculation.  Calibrate by measuring       *
 * full-speed close duration on the bench.                              */

#ifndef SPAWN_CLOSE_LATENCY_MS
#define SPAWN_CLOSE_LATENCY_MS 800u /* ms from close command to flap seated */
#endif

/* ── Finish Mode A: close-early + top-off ───────────────────────────── *
 * Close when:  remaining_ug < predicted_in_flight_ug + CLOSE_EARLY_MARGIN_UG *
 * predicted_in_flight_ug = (ema_flow_ug_per_tick * SPAWN_CLOSE_LATENCY_MS) /
 * SPAWN_TIMER_PERIOD_MS */

#ifndef SPAWN_CLOSE_EARLY_MARGIN_UG
#define SPAWN_CLOSE_EARLY_MARGIN_UG                                            \
  500000u /* extra safety margin (µg) for early close */
#endif
/* Top-off: issue small open pulses if final mass < target - TOPOFF_TOLERANCE_UG
 */
#ifndef SPAWN_TOPOFF_TOLERANCE_UG
#define SPAWN_TOPOFF_TOLERANCE_UG                                              \
  1000000u /* undershoot tolerance for top-off (µg = 1 g) */
#endif
#ifndef SPAWN_TOPOFF_PULSE_MS
#define SPAWN_TOPOFF_PULSE_MS 80u /* duration of each top-off open nudge (ms)  \
                                   */
#endif
#ifndef SPAWN_TOPOFF_MAX_PULSES
#define SPAWN_TOPOFF_MAX_PULSES                                                \
  5u /* max top-off attempts before declaring done */
#endif
/* Pause between top-off open and re-close to let weight settle (ms) */
#ifndef SPAWN_TOPOFF_SETTLE_MS
#define SPAWN_TOPOFF_SETTLE_MS 200u /* settle time after top-off pulse (ms) */
#endif

/* ── Finish Mode B: low-flow taper ─────────────────────────────────── *
 * When remaining_ug < LOWFLOW_THRESHOLD_UG, switch to a minimal open  *
 * nudge duration so the flap barely trickles.                          */

#ifndef SPAWN_LOWFLOW_THRESHOLD_UG
#define SPAWN_LOWFLOW_THRESHOLD_UG                                             \
  5000000u /* enter low-flow taper when ≤ 5 g remain */
#endif
#ifndef SPAWN_LOWFLOW_NUDGE_MS
#define SPAWN_LOWFLOW_NUDGE_MS                                                 \
  80u /* reduced nudge duration in low-flow phase (ms) */
#endif
/* Close fully when remaining_ug < CLOSE_THRESHOLD_UG (within low-flow phase) */
#ifndef SPAWN_CLOSE_THRESHOLD_UG
#define SPAWN_CLOSE_THRESHOLD_UG                                               \
  500000u /* 0.5 g — issue final close from low-flow */
#endif

/* ================================================================== */
/*  Agitator stepper (DRV8434S, STEPPER_DEV_AGITATOR when wired)      */
/* ================================================================== */

#ifndef SPAWN_MAX_AGITATE_RETRIES
#define SPAWN_MAX_AGITATE_RETRIES                                              \
  5 /* max agitation attempts before SPAWN_STATUS_BAG_EMPTY */
#endif
#ifndef AGITATOR_KNEAD_STEPS
#define AGITATOR_KNEAD_STEPS 200 /* steps per agitation knead cycle */
#endif
#ifndef AGITATOR_STEP_DELAY_US
#define AGITATOR_STEP_DELAY_US 2000u /* µs per step (adjust for torque) */
#endif

/* ================================================================== */
/*  Hot wire carriage traverse (DRV8434S, STEPPER_DEV_HW_CARRIAGE)    */
/* ================================================================== */

#ifndef HOTWIRE_TRAVERSE_STEPS
#define HOTWIRE_TRAVERSE_STEPS 1000 /* steps to traverse full cut distance */
#endif
#ifndef HOTWIRE_TRAVERSE_STEP_DELAY_US
#define HOTWIRE_TRAVERSE_STEP_DELAY_US 2000u /* µs per step */
#endif

/* ================================================================== */
/*  Indexer / bag depth rack (DRV8434S, STEPPER_DEV_INDEXER)          */
/* ================================================================== */

#ifndef INDEXER_STEPS_CENTER
#define INDEXER_STEPS_CENTER                                                   \
  3000 /* steps from home to bag-centering position */
#endif
#ifndef INDEXER_STEPS_EJECT
#define INDEXER_STEPS_EJECT                                                    \
  8000 /* steps from home to full bag-eject position */
#endif
