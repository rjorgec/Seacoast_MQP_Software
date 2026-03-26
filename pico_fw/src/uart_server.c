#include "uart_server.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/spi.h"
#include "hardware/uart.h"

#include "board_pins.h"
#include "drivers/drv8163/drv8163.h"
#include "drivers/drv8434s/drv8434s.h"

#include "extern/pico-scale/include/hx711_scale_adaptor.h"
#include "extern/pico-scale/include/scale.h"
#include "extern/pico-scale/extern/hx711-pico-c/include/common.h"

#include "shared/proto/proto.h"
#include "shared/proto/cobs.h"

/* ── Frame-level constants ─────────────────────────────────────────────────── */

#define UART_RX_RING_SIZE 512u
#define UART_ENCODED_FRAME_MAX 256u
#define UART_DECODED_FRAME_MAX ((size_t)sizeof(proto_hdr_t) + PROTO_MAX_PAYLOAD + 2u)

/* Default step delay used by the high-level stepper handlers (µs per step). */
#define STEPPER_DEFAULT_STEP_DELAY_US 1000u

/* ── Ring buffer ───────────────────────────────────────────────────────────── */

typedef struct
{
    uint8_t buf[UART_RX_RING_SIZE];
    uint16_t head;
    uint16_t tail;
    uint16_t count;
} rx_ring_t;

static rx_ring_t s_rx_ring;
static uint8_t s_frame_buf[UART_ENCODED_FRAME_MAX];
static size_t s_frame_len;
static bool s_frame_overflow;

/* ── DRV8163 — flap (primary) ──────────────────────────────────────────────── */

static drv8163_t s_drv8163;
static bool s_drv8163_ready;

/* ── DRV8163 — flap (second instance) ─────────────────────────────────────── */

static drv8163_t s_drv8163_flap2;
static bool s_drv8163_flap2_ready = false;

/* ── DRV8163 — hot wire (secondary) ───────────────────────────────────────── */

static drv8163_t s_drv8163_hotwire;
static bool s_drv8163_hotwire_ready;

/* ── DRV8434S stepper chain & motion engine ────────────────────────────────── */

static drv8434s_chain_t s_stepper_chain;
static drv8434s_motion_t s_motion;
static struct repeating_timer s_step_timer;
static bool s_step_timer_active;
static bool s_stepper_ready;

/* ── UART ──────────────────────────────────────────────────────────────────── */

static uart_inst_t *s_uart;
static bool s_uart_ready;

/* ── HX711 load cell ───────────────────────────────────────────────────────── */

static hx711_t s_hx711;
static bool s_hx711_ready;
static hx711_scale_adaptor_t s_hxsa;
static scale_t s_scale;
static scale_options_t s_scale_opt;
static int32_t s_scale_valbuff[128]; /* 80 SPS max × ~1 s worst-case read window */
static mass_unit_t unit = mass_g;
static int32_t refUnit = -165;   // slope
static int32_t offset = 8130430; // offset

/* ── Flap state machine ────────────────────────────────────────────────────── */

typedef enum
{
    FLAP_SM_IDLE = 0,
    FLAP_SM_OPENING,
    FLAP_SM_CLOSING,
    FLAP_SM_OPEN,
    FLAP_SM_CLOSED,
    FLAP_SM_FAULT,
} flap_sm_state_t;

static flap_sm_state_t s_flap_state;
static uint32_t s_flap_start_ms;
static bool s_flap1_stopped = false; /* flap1 reached its endpoint and was stopped */
static bool s_flap2_stopped = false; /* flap2 reached its endpoint and was stopped */

/* ── Stepper absolute position tracking ────────────────────────────────────── */

static int32_t s_arm_pos_steps = 0;
static int32_t s_rack_pos_steps = 0;
static int32_t s_turntable_pos_steps = 0;
static bool s_turntable_homed = false;

/* ── Per-device pending motion tracking ────────────────────────────────────── */

typedef struct
{
    bool pending;
    int32_t target_steps;
    subsystem_id_t subsys;
    uint32_t start_ms;
    uint32_t timeout_ms;
    uint8_t dev_idx;
} stepper_pending_t;

static stepper_pending_t s_arm_pending;
static stepper_pending_t s_rack_pending;
static stepper_pending_t s_turntable_pending;

/* ── Vacuum pump state ─────────────────────────────────────────────────────── */

static bool s_vacuum_on = false;
static bool s_vacuum2_on = false;
static volatile uint32_t s_vacuum_pulse_count = 0; /* written by GPIO ISR */
static uint32_t s_vacuum_last_sample_ms = 0;
static uint32_t s_vacuum_last_status_ms = 0;
static vacuum_status_code_t s_vacuum_status = VACUUM_OFF;

/* ── Dosing callback ─────────────────────────────────────────────────────── */

typedef struct
{
    bool active;
    bool agitating;
    bool startup_opening; /* true while driving open before the first weight change */
    uint8_t retry;
    uint8_t max_retries;
    uint16_t bag_number;
    uint32_t start_mass_ug;
    uint32_t target_ug;
    uint32_t dispensed_ug;
    uint32_t window_start_ug; /* dispensed_ug at start of current 500 ms flow window */
    uint32_t last_tick_ug;    /* dispensed_ug at the previous tick for nudge delta */
    uint32_t spawn_remaining_ug;
    absolute_time_t last_flow_check; /* timestamp of current window start */
    repeating_timer_t timer;
    bool nudging;                /* true while a position-nudge pulse is active */
    absolute_time_t nudge_until; /* absolute time when the current nudge should end */
} spawn_dose_ctx_t;

static spawn_dose_ctx_t s_spawn;

#define SPAWN_TIMER_PERIOD_MS 50
#define SPAWN_FLOW_WINDOW_MS 100
#define SPAWN_TICKS_PER_WINDOW (SPAWN_FLOW_WINDOW_MS / SPAWN_TIMER_PERIOD_MS)
#define SPAWN_SCALE_READ_SAMPLES 1u   /* Single shot reading */
#define SPAWN_FLOW_NOFLOW_UG 0500000u /* minimum mass per windoow to consider flowing*/
#define SPAWN_FLOW_MIN_UG 1000000u    /* mass per window to target when completing dose */
#define SPAWN_FLOW_MAX_UG 5000000u    /* mass per window to target when starting dose */
#define SPAWN_MAX_RETRIES 100
#define SPAWN_AGITATE_MS 2000
/* Per-tick flow targets (window totals divided by ticks per window) */
#define SPAWN_TICK_MIN_UG (SPAWN_FLOW_MIN_UG / SPAWN_TICKS_PER_WINDOW) /* 1000 ug */
#define SPAWN_TICK_MAX_UG (SPAWN_FLOW_MAX_UG / SPAWN_TICKS_PER_WINDOW) /* 4000 ug */
#define SPAWN_TICK_DEADBAND 0050000u                                   /* ±300 ug/tick — no nudge needed within this band */
/* Flap PWM values used during dosing */
#define SPAWN_OPEN_PWM (FLAP_OPEN_SPEED_PWM / 2)     /* forward PWM for startup opening and open nudges */
#define SPAWN_REVERSE_PWM (FLAP_CLOSE_SPEED_PWM / 2) /* reverse nudge PWM */
/* Minimum weight increase per 500 ms tick to consider material flowing during startup opening */
#define SPAWN_STARTUP_FLOW_DETECT_UG 1000000u /* 100 mg */
/* Duration of each position nudge — must be shorter than SPAWN_TIMER_PERIOD_MS */
#define SPAWN_NUDGE_OPEN_MS 300u  /* ms to nudge open per tick */
#define SPAWN_NUDGE_CLOSE_MS 200u /* ms to nudge closed per tick */
/* Proportional scaling of the per-tick target rate:
 *   remaining > target / SPAWN_PROP_UPPER → SPAWN_TICK_MAX_UG target rate
 *   remaining < target / SPAWN_PROP_LOWER → SPAWN_TICK_MIN_UG target rate
 *   in between                            → linear interpolation */
#define SPAWN_PROP_UPPER 2u  /* 50 % of target remaining */
#define SPAWN_PROP_LOWER 10u /* 10 % of target remaining */
#define SPAWN_INNOC_PCT_MIN_X10 10u
#define SPAWN_INNOC_PCT_MAX_X10 500u

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  Frame transmit helpers                                                     */
/* ═══════════════════════════════════════════════════════════════════════════ */

static bool uart_send_frame(uint8_t type, uint16_t seq, const void *payload, uint16_t len)
{
    uint8_t raw[UART_DECODED_FRAME_MAX];
    uint8_t enc[UART_ENCODED_FRAME_MAX];

    if (len > PROTO_MAX_PAYLOAD)
    {
        return false;
    }

    proto_hdr_t hdr = {
        .version = PROTO_VERSION,
        .type = type,
        .seq = seq,
        .len = len,
    };

    memcpy(raw, &hdr, sizeof(hdr));
    if (payload != NULL && len > 0u)
    {
        memcpy(raw + sizeof(hdr), payload, len);
    }

    const size_t raw_len = sizeof(hdr) + len + 2u;
    uint16_t crc = proto_crc16_ccitt(raw, (uint32_t)(sizeof(hdr) + len));
    raw[sizeof(hdr) + len] = (uint8_t)(crc & 0xFFu);
    raw[sizeof(hdr) + len + 1u] = (uint8_t)((crc >> 8) & 0xFFu);

    const size_t enc_len = cobs_encode(raw, raw_len, enc);
    if (enc_len == 0u || enc_len > sizeof(enc))
    {
        return false;
    }

    uart_write_blocking(s_uart, enc, enc_len);
    const uint8_t delim = PROTO_DELIM;
    uart_write_blocking(s_uart, &delim, 1u);
    return true;
}

static void send_ack(uint16_t seq)
{
    (void)uart_send_frame(MSG_ACK, seq, NULL, 0u);
}

static void send_nack(uint16_t seq, nack_code_t code)
{
    pl_nack_t nack = {.code = (uint8_t)code};
    (void)uart_send_frame(MSG_NACK, seq, &nack, (uint16_t)sizeof(nack));
}

/* ── Unsolicited outbound helpers ───────────────────────────────────────────── */

/**
 * @brief Send MSG_MOTION_DONE (unsolicited, seq=0) to the ESP32.
 */
static void send_motion_done(subsystem_id_t subsys, motion_result_t result, int32_t steps_done)
{
    pl_motion_done_t pl = {
        .subsystem = (uint8_t)subsys,
        .result = (uint8_t)result,
        ._rsvd = {0u, 0u},
        .steps_done = steps_done,
    };
    (void)uart_send_frame(MSG_MOTION_DONE, 0u, &pl, (uint16_t)sizeof(pl));
}

/**
 * @brief Send MSG_VACUUM_STATUS (unsolicited, seq=0) to the ESP32.
 */
static void send_vacuum_status(vacuum_status_code_t status, uint16_t rpm)
{
    pl_vacuum_status_t pl = {
        .status = (uint8_t)status,
        ._rsvd = 0u,
        .rpm = rpm,
    };
    (void)uart_send_frame(MSG_VACUUM_STATUS, 0u, &pl, (uint16_t)sizeof(pl));
}

/* ── Spawn dosing helpers ─────────────────────────────────────────────────── */

/**
 * @brief Shared internal helper: configure both flap drivers for a torque-monitored
 *        close and engage the flap state machine.  Used by both handle_flap_close()
 *        and spawn_close_flaps() so the logic is never duplicated.
 *
 * @return true if monitoring started successfully, false on driver error.
 */
static bool flap_close_internal(void)
{
    if (!s_drv8163_ready)
    {
        return false;
    }

    if (s_drv8163.monitoring_enabled)
    {
        drv8163_stop_current_monitoring(&s_drv8163);
    }
    s_drv8163.config.current_low_threshold = FLAP_OPEN_CURRENT_DROP_TH;
    s_drv8163.config.current_high_threshold = (uint16_t)FLAP_CLOSE_TORQUE_TH;
    s_drv8163.config.current_check_interval_ms = (uint32_t)FLAP_MONITOR_INTERVAL_MS;
    (void)drv8163_set_motor_control(&s_drv8163, DRV8163_MOTOR_REVERSE,
                                    (uint16_t)FLAP_CLOSE_SPEED_PWM);
    if (!drv8163_start_current_monitoring(&s_drv8163))
    {
        (void)drv8163_set_motor_control(&s_drv8163, DRV8163_MOTOR_STOP, 0u);
        return false;
    }

    if (s_drv8163_flap2_ready)
    {
        if (s_drv8163_flap2.monitoring_enabled)
        {
            drv8163_stop_current_monitoring(&s_drv8163_flap2);
        }
        s_drv8163_flap2.config.current_low_threshold = FLAP_OPEN_CURRENT_DROP_TH;
        s_drv8163_flap2.config.current_high_threshold = (uint16_t)FLAP_CLOSE_TORQUE_TH;
        s_drv8163_flap2.config.current_check_interval_ms = (uint32_t)FLAP_MONITOR_INTERVAL_MS;
        (void)drv8163_set_motor_control(&s_drv8163_flap2, DRV8163_MOTOR_REVERSE,
                                        (uint16_t)FLAP_CLOSE_SPEED_PWM);
        (void)drv8163_start_current_monitoring(&s_drv8163_flap2);
    }

    s_flap_state = FLAP_SM_CLOSING;
    s_flap_start_ms = to_ms_since_boot(get_absolute_time());
    s_flap1_stopped = false;
    s_flap2_stopped = (!s_drv8163_flap2_ready);
    return true;
}

/**
 * @brief Stop both flap motors immediately and halt any active current monitoring.
 *        Called during spawn dosing to immediately arrest motion before transitioning.
 */
static void spawn_stop_flaps(void)
{
    if (s_drv8163.monitoring_enabled)
    {
        drv8163_stop_current_monitoring(&s_drv8163);
    }
    (void)drv8163_set_motor_control(&s_drv8163, DRV8163_MOTOR_STOP, 0u);

    if (s_drv8163_flap2_ready)
    {
        if (s_drv8163_flap2.monitoring_enabled)
        {
            drv8163_stop_current_monitoring(&s_drv8163_flap2);
        }
        (void)drv8163_set_motor_control(&s_drv8163_flap2, DRV8163_MOTOR_STOP, 0u);
    }
    s_spawn.nudging = false;
}

/**
 * @brief Close the flaps after spawn dosing is complete or aborted.
 *        Stops any ongoing monitoring first, then engages the torque-monitored close.
 */
static void spawn_close_flaps(void)
{
    spawn_stop_flaps();
    (void)flap_close_internal();
}

/**
 * @brief Set the flap motor(s) to a continuous PWM without engaging the
 *        torque/undercurrent state machine.  Used for proportional dosing control.
 *
 * @param forward  true = open direction, false = close direction.
 * @param pwm      PWM duty 0..4095.
 */
static void spawn_set_flap_pwm(bool forward, uint16_t pwm)
{
    drv8163_motor_state_t st = forward ? DRV8163_MOTOR_FORWARD : DRV8163_MOTOR_REVERSE;
    (void)drv8163_set_motor_control(&s_drv8163, st, pwm);
    if (s_drv8163_flap2_ready)
    {
        (void)drv8163_set_motor_control(&s_drv8163_flap2, st, pwm);
    }
}

static void send_spawn_status(spawn_status_code_t status)
{
    pl_spawn_status_t pl = {
        .status = (uint8_t)status,
        .retries = s_spawn.retry,
        .bag_number = s_spawn.bag_number,
        .target_ug = s_spawn.target_ug,
        .disp_ug = s_spawn.dispensed_ug,
        .remain_ug = s_spawn.spawn_remaining_ug,
    };

    (void)uart_send_frame(MSG_SPAWN_STATUS, 0u, &pl, (uint16_t)sizeof(pl));
}

static void spawn_stop_timer(void)
{
    (void)cancel_repeating_timer(&s_spawn.timer);
    s_spawn.timer.delay_us = 0;
}

static bool dispense_spawn_callback(struct repeating_timer *t)
{
    (void)t;
    if (!s_spawn.active)
    {
        return false;
    }

    absolute_time_t now = get_absolute_time();

    /* ── Nudge expiry: stop motor when position nudge time has elapsed ──────── */
    if (s_spawn.nudging &&
        absolute_time_diff_us(s_spawn.nudge_until, now) >= 0)
    {
        /* Stop motor; hold current flap position until next nudge decision */
        spawn_set_flap_pwm(true, 0u); /* coast = no drive in either direction */
        (void)drv8163_set_motor_control(&s_drv8163, DRV8163_MOTOR_STOP, 0u);
        if (s_drv8163_flap2_ready)
        {
            (void)drv8163_set_motor_control(&s_drv8163_flap2, DRV8163_MOTOR_STOP, 0u);
        }
        s_spawn.nudging = false;
    }

    /* ── Scale read — drain and average all queued HX711 FIFO entries ─────── *
     * Between 500ms ticks the PIO FIFO accumulates up to 8 stale readings     *
     * (RP2350 joined TX+RX FIFOs, SM stalls when full).  hx711_get_value_     *
     * noblock pops all available entries immediately without blocking; every   *
     * collected reading is averaged, giving a noise-tolerant estimate that     *
     * reflects the weight at the END of the inter-tick window (most recent    *
     * readings dominate once the FIFO overflows).  If the FIFO is empty a     *
     * single blocking hx711_get_value forces one fresh 12.5ms conversion.     *
     * Calibration is applied directly from s_scale (tare-aware).              */
    mass_t mass = {0};
    {
        int32_t fifo_buf[8];
        size_t fifo_count = 0u;
        int32_t raw_val;
        while (fifo_count < 8u && hx711_get_value_noblock(&s_hx711, &raw_val))
        {
            fifo_buf[fifo_count++] = raw_val;
        }
        if (fifo_count == 0u)
        {
            fifo_buf[0] = hx711_get_value(&s_hx711); /* block for one fresh sample */
            fifo_count = 1u;
        }
        int64_t raw_sum = 0;
        for (size_t i = 0u; i < fifo_count; ++i)
        {
            raw_sum += (int64_t)fifo_buf[i];
        }
        double raw_avg = (double)raw_sum / (double)fifo_count;
        /* mass_g = (raw − offset) / ref_unit  →  mass_ug = mass_g × 1 000 000 */
        double mass_g = (raw_avg - (double)s_scale.offset) / (double)s_scale.ref_unit;
        mass.ug = (int32_t)(mass_g * 1000000.0);
        mass.unit = s_scale.unit;
    }

    uint32_t current_ug = (uint32_t)mass.ug;
    double m = 0;
    mass_get_value(&mass, &m);
    printf("Mass (g) %f\n", m);
    s_spawn.dispensed_ug = (current_ug > s_spawn.start_mass_ug)
                               ? (current_ug - s_spawn.start_mass_ug)
                               : 0u;

    /* ── Per-tick delta — computed before undercurrent check ────────────────── */
    uint32_t tick_delta_ug = (s_spawn.dispensed_ug > s_spawn.last_tick_ug)
                                 ? (s_spawn.dispensed_ug - s_spawn.last_tick_ug)
                                 : 0u;
    s_spawn.last_tick_ug = s_spawn.dispensed_ug;

    /* ── Flaps fully-open detection ────────────────────────────────────────── *
     * Undercurrent = no mechanical load = flap at open-circuit endpoint.       *
     * During startup_opening: only a failure if no weight change yet —         *
     *   if material is already flowing the flap position is acceptable.        *
     * During nudging: undercurrent at any time signals end-of-travel.          *
     * Guard: when motor is stopped between nudges current is zero regardless   *
     *   of position, so only evaluate when actively driving forward.           */
    bool startup_undercurrent = s_spawn.startup_opening &&
                                s_drv8163.current_status == DRV8163_CURRENT_UNDERCURRENT &&
                                tick_delta_ug < SPAWN_STARTUP_FLOW_DETECT_UG;
    bool nudge_undercurrent = s_spawn.nudging &&
                              s_drv8163.current_status == DRV8163_CURRENT_UNDERCURRENT;
    if (startup_undercurrent || nudge_undercurrent)
    {
        printf("Spawn: flaps fully open (undercurrent) — flow failure\n");
        spawn_close_flaps();
        s_spawn.active = false;
        send_spawn_status(SPAWN_STATUS_FLOW_FAILURE);
        return false;
    }

    /* ── Target reached ─────────────────────────────────────────────────────── */
    if (s_spawn.dispensed_ug >= s_spawn.target_ug)
    {
        spawn_close_flaps();
        s_spawn.active = false;
        send_spawn_status(SPAWN_STATUS_DONE);
        return false;
    }

    /* ── Startup opening phase ───────────────────────────────────────────────── *
     * The flap is driven open at full PWM while waiting for the first           *
     * SPAWN_STARTUP_FLOW_DETECT_UG weight increase that confirms material is    *
     * flowing.  Undercurrent detection (checked above) fires first if the flap  *
     * reaches end-of-travel before any weight change, signalling a blocked or   *
     * empty outlet.  Once flow is confirmed, monitoring is stopped (no longer   *
     * needed), the motor is parked, and control hands off to the nudge loop.    */
    if (s_spawn.startup_opening)
    {
        if (tick_delta_ug >= SPAWN_STARTUP_FLOW_DETECT_UG)
        {
            /* Flow confirmed — stop undercurrent monitoring, park motor,
             * reset the flow window, and fall through to the nudge controller. */
            drv8163_stop_current_monitoring(&s_drv8163);
            if (s_drv8163_flap2_ready && s_drv8163_flap2.monitoring_enabled)
            {
                drv8163_stop_current_monitoring(&s_drv8163_flap2);
            }
            s_spawn.startup_opening = false;
            s_spawn.nudging = false;
            (void)drv8163_set_motor_control(&s_drv8163, DRV8163_MOTOR_STOP, 0u);
            if (s_drv8163_flap2_ready)
            {
                (void)drv8163_set_motor_control(&s_drv8163_flap2, DRV8163_MOTOR_STOP, 0u);
            }
            /* Reset flow window so the 500 ms budget begins from first flow. */
            s_spawn.last_flow_check = get_absolute_time();
            s_spawn.window_start_ug = s_spawn.dispensed_ug;
            printf("Spawn: startup opening complete — flow detected (%lu ug), entering closed loop\n",
                   (unsigned long)tick_delta_ug);
            /* Fall through to nudge logic below */
        }
        else
        {
            /* No weight change yet — keep driving open continuously */
            spawn_set_flap_pwm(true, (uint16_t)SPAWN_OPEN_PWM);
            return true;
        }
    }

    /* ── 500 ms flow-rate window (agitation / retry logic) ──────────────────── */
    now = get_absolute_time();
    if (absolute_time_diff_us(s_spawn.last_flow_check, now) >= (int64_t)(SPAWN_FLOW_WINDOW_MS * 1000))
    {
        uint32_t window_delta = (s_spawn.dispensed_ug > s_spawn.window_start_ug)
                                    ? (s_spawn.dispensed_ug - s_spawn.window_start_ug)
                                    : 0u;

        if (!s_spawn.agitating && window_delta < SPAWN_FLOW_NOFLOW_UG)
        {
            /* Insufficient flow — always agitate before checking abort so that
             * the agitation actually runs.  Abort is evaluated after the
             * agitation window elapses (see post-agitation block below). */
            spawn_stop_flaps();
            s_spawn.agitating = true;
            s_spawn.retry++;
            send_spawn_status(SPAWN_STATUS_AGITATING);
            /* Hold off the next window check by SPAWN_AGITATE_MS */
            s_spawn.last_flow_check = delayed_by_ms(now, SPAWN_AGITATE_MS);
            s_spawn.window_start_ug = s_spawn.dispensed_ug;
            return true;
        }

        /* Window completed normally — reset window tracking */
        s_spawn.window_start_ug = s_spawn.dispensed_ug;
        s_spawn.last_flow_check = now;

        if (s_spawn.agitating)
        {
            /* Agitation period has elapsed — check retry limit, then re-open. */
            s_spawn.agitating = false;
            if (s_spawn.retry >= s_spawn.max_retries)
            {
                s_spawn.active = false;
                send_spawn_status(SPAWN_STATUS_BAG_EMPTY);
                return false;
            }
            /* Re-engage the startup opening sequence so the flap drives open
             * again and waits for the first weight change before handing off
             * to the closed-loop nudge controller — same as initial start. */
            s_spawn.startup_opening = true;
            s_spawn.last_flow_check = get_absolute_time();
            s_spawn.window_start_ug = s_spawn.dispensed_ug;
            s_spawn.last_tick_ug = s_spawn.dispensed_ug;

            s_drv8163.config.current_low_threshold = (uint16_t)FLAP_OPEN_CURRENT_DROP_TH;
            s_drv8163.config.current_high_threshold = 4095u;
            s_drv8163.config.current_check_interval_ms = (uint32_t)FLAP_MONITOR_INTERVAL_MS;
            spawn_set_flap_pwm(true, (uint16_t)SPAWN_OPEN_PWM);
            (void)drv8163_start_current_monitoring(&s_drv8163);

            if (s_drv8163_flap2_ready)
            {
                s_drv8163_flap2.config.current_low_threshold = (uint16_t)FLAP_OPEN_CURRENT_DROP_TH;
                s_drv8163_flap2.config.current_high_threshold = 4095u;
                s_drv8163_flap2.config.current_check_interval_ms = (uint32_t)FLAP_MONITOR_INTERVAL_MS;
                (void)drv8163_start_current_monitoring(&s_drv8163_flap2);
            }

            send_spawn_status(SPAWN_STATUS_RUNNING);
        }
    }

    /* During agitation hold-off keep the flaps stopped */
    if (s_spawn.agitating)
    {
        return true;
    }

    /* ── Proportional per-tick target rate ───────────────────────────────────── *
     *                                                                            *
     * Scale the desired flow rate by how much is left to dispense.  When far    *
     * from the target a higher rate is acceptable; as we near the target the     *
     * rate tapers to SPAWN_TICK_MIN_UG to avoid overshoot.                       *
     *                                                                            *
     *   remaining > target / SPAWN_PROP_UPPER → desired = SPAWN_TICK_MAX_UG     *
     *   remaining < target / SPAWN_PROP_LOWER → desired = SPAWN_TICK_MIN_UG     *
     *   in between                            → linear interpolation             *
     * ──────────────────────────────────────────────────────────────────────────── */
    uint32_t remaining_ug = s_spawn.target_ug - s_spawn.dispensed_ug;
    uint32_t upper_ug = s_spawn.target_ug / SPAWN_PROP_UPPER;
    uint32_t lower_ug = s_spawn.target_ug / SPAWN_PROP_LOWER;
    uint32_t desired_tick_ug;

    if (remaining_ug >= upper_ug)
    {
        desired_tick_ug = SPAWN_TICK_MAX_UG;
    }
    else if (remaining_ug > lower_ug)
    {
        uint32_t range = upper_ug - lower_ug;
        uint32_t frac = remaining_ug - lower_ug;
        desired_tick_ug = SPAWN_TICK_MIN_UG +
                          ((uint32_t)(SPAWN_TICK_MAX_UG - SPAWN_TICK_MIN_UG) * frac) / range;
    }
    else
    {
        desired_tick_ug = SPAWN_TICK_MIN_UG;
    }

    /* ── Incremental position nudge ─────────────────────────────────────────── *
     *                                                                            *
     * The flap position (not speed) is what controls flow rate.  Rather than    *
     * driving the motor continuously — which would keep opening the flap more   *
     * and more — we issue short timed nudges to increment position up or down   *
     * based on whether actual per-tick flow is below or above the target.        *
     * Between nudges the motor is stopped, holding flap position.               *
     * ──────────────────────────────────────────────────────────────────────────── */
    if (!s_spawn.nudging) /* only issue a new nudge when the previous one has expired */
    {
        /* Cast to int32 to handle the signed error correctly */
        int32_t error_ug = (int32_t)desired_tick_ug - (int32_t)tick_delta_ug;

        if (error_ug > (int32_t)SPAWN_TICK_DEADBAND)
        {
            /* Flow below target — open flap a little */
            spawn_set_flap_pwm(true, (uint16_t)SPAWN_OPEN_PWM);
            s_spawn.nudge_until = delayed_by_ms(get_absolute_time(), SPAWN_NUDGE_OPEN_MS);
            s_spawn.nudging = true;
        }
        else if (error_ug < -(int32_t)SPAWN_TICK_DEADBAND)
        {
            /* Flow above target — close flap a little */
            spawn_set_flap_pwm(false, (uint16_t)SPAWN_REVERSE_PWM);
            s_spawn.nudge_until = delayed_by_ms(get_absolute_time(), SPAWN_NUDGE_CLOSE_MS);
            s_spawn.nudging = true;
        }
        /* else: within deadband — hold position (motor already stopped) */
    }

    return true;
}

/* ── DRV8434S SPI callbacks ─────────────────────────────────────────────────── */

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
    sleep_us(us);
}

static void stepper_delay_ms(void *ctx, unsigned ms)
{
    (void)ctx;
    sleep_ms(ms);
}

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  Ring buffer                                                                */
/* ═══════════════════════════════════════════════════════════════════════════ */

static bool rx_ring_push(uint8_t b)
{
    if (s_rx_ring.count >= UART_RX_RING_SIZE)
    {
        return false;
    }

    s_rx_ring.buf[s_rx_ring.head] = b;
    s_rx_ring.head = (uint16_t)((s_rx_ring.head + 1u) % UART_RX_RING_SIZE);
    ++s_rx_ring.count;
    return true;
}

static bool rx_ring_pop(uint8_t *out)
{
    if (s_rx_ring.count == 0u || out == NULL)
    {
        return false;
    }

    *out = s_rx_ring.buf[s_rx_ring.tail];
    s_rx_ring.tail = (uint16_t)((s_rx_ring.tail + 1u) % UART_RX_RING_SIZE);
    --s_rx_ring.count;
    return true;
}

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  Vacuum RPM ISR                                                             */
/* ═══════════════════════════════════════════════════════════════════════════ */

static void vacuum_rpm_isr(uint gpio, uint32_t events)
{
    (void)events;
    if (gpio == (uint)VACUUM_RPM_SENSE_PIN)
    {
        ++s_vacuum_pulse_count;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  DRV8434S stepper motion engine callbacks & timer                           */
/* ═══════════════════════════════════════════════════════════════════════════ */

static void stepper_motion_done(void *ctx, uint8_t dev_idx,
                                const drv8434s_motion_result_t *result)
{
    (void)ctx;
    printf("Stepper[%u]: done — %li of %li steps, torque=%u, reason=%u\n",
           dev_idx,
           (long)result->steps_achieved,
           (long)result->steps_requested,
           result->last_torque_count,
           (unsigned)result->reason);
    /* step_timer_callback returns false on this same tick when no jobs remain,
     * which cancels the repeating timer automatically. */
}

static bool step_timer_callback(struct repeating_timer *t)
{
    (void)t;
    drv8434s_motion_tick(&s_motion);

    /* Return true to keep the timer running; false to cancel. */
    if (!drv8434s_motion_is_busy(&s_motion))
    {
        s_step_timer_active = false;
        return false;
    }
    return true;
}

/* ── Internal helper: ensure step timer is running ──────────────────────────── */

static bool ensure_step_timer_running(void)
{
    if (s_step_timer_active)
    {
        return true;
    }
    if (!add_repeating_timer_us(-(int64_t)STEPPER_DEFAULT_STEP_DELAY_US,
                                step_timer_callback, NULL, &s_step_timer))
    {
        return false;
    }
    s_step_timer_active = true;
    return true;
}

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  Polling tick functions (called from uart_server_poll)                      */
/* ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Flap state-machine tick — detects current threshold crossings.
 *
 * The DRV8163 monitoring timer updates s_drv8163.current_status in ISR
 * context.  We poll it here and fire the MOTION_DONE notification when a
 * threshold crossing or timeout is detected.
 */
static void flap_sm_tick(void)
{
    if (s_flap_state != FLAP_SM_OPENING && s_flap_state != FLAP_SM_CLOSING)
    {
        return;
    }

    uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    bool timed_out = (now_ms - s_flap_start_ms) >= (uint32_t)FLAP_MOTION_TIMEOUT_MS;

    if (s_flap_state == FLAP_SM_OPENING)
    {
        /* Stop each flap individually when it reaches the open-circuit endpoint */
        if (!s_flap1_stopped &&
            s_drv8163.current_status == DRV8163_CURRENT_UNDERCURRENT)
        {
            drv8163_stop_current_monitoring(&s_drv8163);
            (void)drv8163_set_motor_control(&s_drv8163, DRV8163_MOTOR_STOP, 0u);
            s_flap1_stopped = true;
            printf("Flap1 OPEN: reached endpoint (current drop)\n");
        }

        if (!s_flap2_stopped && s_drv8163_flap2_ready &&
            s_drv8163_flap2.current_status == DRV8163_CURRENT_UNDERCURRENT)
        {
            drv8163_stop_current_monitoring(&s_drv8163_flap2);
            (void)drv8163_set_motor_control(&s_drv8163_flap2, DRV8163_MOTOR_STOP, 0u);
            s_flap2_stopped = true;
            printf("Flap2 OPEN: reached endpoint (current drop)\n");
        }

        /* Check overall completion */
        if (s_flap1_stopped && s_flap2_stopped)
        {
            printf("Flap OPEN: both actuators stopped\n");
            s_flap_state = FLAP_SM_OPEN;
            send_motion_done(SUBSYS_FLAPS, MOTION_OK, 0);
        }
        else if (timed_out)
        {
            /* Stop any still-running actuators */
            if (!s_flap1_stopped)
            {
                drv8163_stop_current_monitoring(&s_drv8163);
                (void)drv8163_set_motor_control(&s_drv8163, DRV8163_MOTOR_STOP, 0u);
            }
            if (!s_flap2_stopped && s_drv8163_flap2_ready)
            {
                drv8163_stop_current_monitoring(&s_drv8163_flap2);
                (void)drv8163_set_motor_control(&s_drv8163_flap2, DRV8163_MOTOR_STOP, 0u);
            }
            printf("Flap OPEN: timeout\n");
            s_flap_state = FLAP_SM_FAULT;
            send_motion_done(SUBSYS_FLAPS, MOTION_TIMEOUT, 0);
        }
    }
    else /* FLAP_SM_CLOSING */
    {
        /* Stop each flap on EITHER endpoint: overcurrent (torque = hard stop)
         * or undercurrent (no load = open-circuit / past end-of-travel).      */
        bool f1_done = !s_flap1_stopped &&
                       (s_drv8163.current_status == DRV8163_CURRENT_OVERCURRENT ||
                        s_drv8163.current_status == DRV8163_CURRENT_UNDERCURRENT);
        bool f2_done = !s_flap2_stopped && s_drv8163_flap2_ready &&
                       (s_drv8163_flap2.current_status == DRV8163_CURRENT_OVERCURRENT ||
                        s_drv8163_flap2.current_status == DRV8163_CURRENT_UNDERCURRENT);

        /* Either flap reaching its endpoint stops both immediately */
        if (f1_done || f2_done)
        {
            if (!s_flap1_stopped)
            {
                drv8163_stop_current_monitoring(&s_drv8163);
                (void)drv8163_set_motor_control(&s_drv8163, DRV8163_MOTOR_STOP, 0u);
                s_flap1_stopped = true;
                printf("Flap1 CLOSE: stopped (%s)\n",
                       f1_done
                           ? (s_drv8163.current_status == DRV8163_CURRENT_UNDERCURRENT
                                  ? "undercurrent"
                                  : "torque")
                           : "partner endpoint");
            }
            if (!s_flap2_stopped && s_drv8163_flap2_ready)
            {
                drv8163_stop_current_monitoring(&s_drv8163_flap2);
                (void)drv8163_set_motor_control(&s_drv8163_flap2, DRV8163_MOTOR_STOP, 0u);
                s_flap2_stopped = true;
                printf("Flap2 CLOSE: stopped (%s)\n",
                       f2_done
                           ? (s_drv8163_flap2.current_status == DRV8163_CURRENT_UNDERCURRENT
                                  ? "undercurrent"
                                  : "torque")
                           : "partner endpoint");
            }
        }

        /* Check overall completion */
        if (s_flap1_stopped && s_flap2_stopped)
        {
            printf("Flap CLOSE: both actuators stopped\n");
            s_flap_state = FLAP_SM_CLOSED;
            send_motion_done(SUBSYS_FLAPS, MOTION_OK, 0);
        }
        else if (timed_out)
        {
            if (!s_flap1_stopped)
            {
                drv8163_stop_current_monitoring(&s_drv8163);
                (void)drv8163_set_motor_control(&s_drv8163, DRV8163_MOTOR_STOP, 0u);
            }
            if (!s_flap2_stopped && s_drv8163_flap2_ready)
            {
                drv8163_stop_current_monitoring(&s_drv8163_flap2);
                (void)drv8163_set_motor_control(&s_drv8163_flap2, DRV8163_MOTOR_STOP, 0u);
            }
            printf("Flap CLOSE: timeout\n");
            s_flap_state = FLAP_SM_FAULT;
            send_motion_done(SUBSYS_FLAPS, MOTION_TIMEOUT, 0);
        }
    }
}

/**
 * @brief Stepper completion tick — detects per-device motion completion and
 *        sends MSG_MOTION_DONE when each pending job finishes or times out.
 *
 * s_motion.jobs[k].active is cleared by the motion engine (from the step
 * timer ISR context) before the done_cb fires.  s_motion.jobs[k].reason is
 * written before active is cleared, so it is safe to read after seeing
 * active == false.
 */
static void stepper_completion_tick(void)
{
    uint32_t now_ms = to_ms_since_boot(get_absolute_time());

    /* ── ARM — device 0 ─────────────────────────────────────── */
    if (s_arm_pending.pending)
    {
        bool job_done = !s_motion.jobs[0].active;
        bool timeout = (now_ms - s_arm_pending.start_ms) >= s_arm_pending.timeout_ms;

        if (job_done || timeout)
        {
            motion_result_t result;

            if (timeout && !job_done)
            {
                /* Still running but deadline exceeded — cancel it. */
                drv8434s_motion_cancel(&s_motion, 0u, NULL);
                result = MOTION_TIMEOUT;
            }
            else
            {
                drv8434s_motion_stop_reason_t reason = s_motion.jobs[0].reason;
                if (reason == DRV8434S_MOTION_OK)
                {
                    result = MOTION_OK;
                    s_arm_pos_steps = s_arm_pending.target_steps;
                }
                else if (reason == DRV8434S_MOTION_TORQUE_LIMIT)
                {
                    result = MOTION_STALLED;
                }
                else
                {
                    result = MOTION_FAULT;
                }
            }

            send_motion_done(SUBSYS_ARM, result, s_arm_pos_steps);
            s_arm_pending.pending = false;
            printf("Arm motion done: result=%u pos=%li\n",
                   (unsigned)result, (long)s_arm_pos_steps);
        }
    }

    /* ── RACK — device 1 ─────────────────────────────────────── */
    if (s_rack_pending.pending)
    {
        bool job_done = !s_motion.jobs[1].active;
        bool timeout = (now_ms - s_rack_pending.start_ms) >= s_rack_pending.timeout_ms;

        if (job_done || timeout)
        {
            motion_result_t result;

            if (timeout && !job_done)
            {
                drv8434s_motion_cancel(&s_motion, 1u, NULL);
                result = MOTION_TIMEOUT;
            }
            else
            {
                drv8434s_motion_stop_reason_t reason = s_motion.jobs[1].reason;
                if (reason == DRV8434S_MOTION_OK)
                {
                    result = MOTION_OK;
                    s_rack_pos_steps = s_rack_pending.target_steps;
                }
                else if (reason == DRV8434S_MOTION_TORQUE_LIMIT)
                {
                    result = MOTION_STALLED;
                }
                else
                {
                    result = MOTION_FAULT;
                }
            }

            send_motion_done(SUBSYS_RACK, result, s_rack_pos_steps);
            s_rack_pending.pending = false;
            printf("Rack motion done: result=%u pos=%li\n",
                   (unsigned)result, (long)s_rack_pos_steps);
        }
    }

    /* ── TURNTABLE — device 2 ────────────────────────────────── */
    if (s_turntable_pending.pending)
    {
        bool job_done = !s_motion.jobs[2].active;
        bool timeout = (now_ms - s_turntable_pending.start_ms) >= s_turntable_pending.timeout_ms;

        if (job_done || timeout)
        {
            motion_result_t result;

            if (timeout && !job_done)
            {
                drv8434s_motion_cancel(&s_motion, 2u, NULL);
                result = MOTION_TIMEOUT;
            }
            else
            {
                drv8434s_motion_stop_reason_t reason = s_motion.jobs[2].reason;
                if (reason == DRV8434S_MOTION_OK)
                {
                    result = MOTION_OK;
                    s_turntable_pos_steps = s_turntable_pending.target_steps;
                }
                else if (reason == DRV8434S_MOTION_TORQUE_LIMIT)
                {
                    result = MOTION_STALLED;
                }
                else
                {
                    result = MOTION_FAULT;
                }
            }

            send_motion_done(SUBSYS_TURNTABLE, result, s_turntable_pos_steps);
            s_turntable_pending.pending = false;
            printf("Turntable motion done: result=%u pos=%li\n",
                   (unsigned)result, (long)s_turntable_pos_steps);
        }
    }
}

/**
 * @brief Vacuum state-machine tick — samples RPM and sends status updates.
 *
 * Called every uart_server_poll() iteration.  Only active while s_vacuum_on.
 * RPM is sampled every VACUUM_RPM_SAMPLE_MS milliseconds by atomically
 * reading and clearing the ISR-maintained pulse counter.
 * MSG_VACUUM_STATUS is sent on state change and every
 * VACUUM_STATUS_SEND_INTERVAL_MS milliseconds while the pump is running.
 */
static void vacuum_sm_tick(void)
{
    if (!s_vacuum_on)
    {
        return;
    }

    uint32_t now_ms = to_ms_since_boot(get_absolute_time());

    if ((now_ms - s_vacuum_last_sample_ms) >= (uint32_t)VACUUM_RPM_SAMPLE_MS)
    {
        /* Atomically snapshot and reset the ISR pulse counter. */
        uint32_t saved = save_and_disable_interrupts();
        uint32_t pulses = s_vacuum_pulse_count;
        s_vacuum_pulse_count = 0u;
        restore_interrupts(saved);

        /*
         * RPM = (pulses / VACUUM_PULSES_PER_REV) * (60 000 ms/min / sample_ms)
         *     = pulses * 60000 / (VACUUM_RPM_SAMPLE_MS * VACUUM_PULSES_PER_REV)
         */
        uint16_t rpm = (uint16_t)((pulses * 60000UL) /
                                  ((uint32_t)VACUUM_RPM_SAMPLE_MS * (uint32_t)VACUUM_PULSES_PER_REV));

        vacuum_status_code_t new_status =
            (rpm < (uint16_t)VACUUM_RPM_BLOCKED_THRESHOLD) ? VACUUM_BLOCKED : VACUUM_OK;

        bool changed = (new_status != s_vacuum_status);
        s_vacuum_status = new_status;
        s_vacuum_last_sample_ms = now_ms;

        /* Send on change or when the periodic interval expires. */
        bool periodic = (now_ms - s_vacuum_last_status_ms) >= (uint32_t)VACUUM_STATUS_SEND_INTERVAL_MS;
        if (changed || periodic)
        {
            send_vacuum_status(s_vacuum_status, rpm);
            s_vacuum_last_status_ms = now_ms;
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  Existing low-level stepper handlers (unchanged)                            */
/* ═══════════════════════════════════════════════════════════════════════════ */

static void handle_stepper_enable(uint16_t seq, const uint8_t *payload, uint16_t len)
{
    if (len != (uint16_t)sizeof(pl_stepper_enable_t))
    {
        send_nack(seq, NACK_BAD_LEN);
        return;
    }

    if (!s_stepper_ready)
    {
        send_nack(seq, NACK_UNKNOWN);
        return;
    }

    const pl_stepper_enable_t *p = (const pl_stepper_enable_t *)payload;
    bool ok = true;
    for (uint8_t k = 0; k < s_stepper_chain.cfg.n_devices && ok; ++k)
    {
        if (p->enable)
            ok = drv8434s_chain_enable(&s_stepper_chain, k, NULL);
        else
            ok = drv8434s_chain_disable(&s_stepper_chain, k, NULL);
    }
    printf("Stepper: %s all → %s\n", p->enable ? "enable" : "disable",
           ok ? "ok" : "fail");

    ok ? send_ack(seq) : send_nack(seq, NACK_UNKNOWN);
}

static void handle_stepper_stepjob(uint16_t seq, const uint8_t *payload, uint16_t len)
{
    if (len != (uint16_t)sizeof(pl_stepper_stepjob_t))
    {
        send_nack(seq, NACK_BAD_LEN);
        return;
    }

    if (!s_stepper_ready)
    {
        send_nack(seq, NACK_UNKNOWN);
        return;
    }

    const pl_stepper_stepjob_t *p = (const pl_stepper_stepjob_t *)payload;
    int32_t target = (p->dir == 0) ? (int32_t)p->steps : -(int32_t)p->steps;

    printf("Stepper: %lu steps dir=%u delay=%lu us\n",
           (unsigned long)p->steps,
           (unsigned)p->dir,
           (unsigned long)p->step_delay_us);

    /* Start motion on device 0 (non-blocking) */
    if (!drv8434s_motion_start(&s_motion, 0u, target, 0u))
    {
        send_nack(seq, NACK_UNKNOWN);
        return;
    }

    if (!s_step_timer_active)
    {
        if (!add_repeating_timer_us(-(int64_t)p->step_delay_us,
                                    step_timer_callback, NULL, &s_step_timer))
        {
            drv8434s_motion_cancel(&s_motion, 0u, NULL);
            send_nack(seq, NACK_UNKNOWN);
            return;
        }
        s_step_timer_active = true;
    }

    send_ack(seq);
}

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  Existing low-level DRV8163 handlers (unchanged)                            */
/* ═══════════════════════════════════════════════════════════════════════════ */

static void handle_drv8163_start(uint16_t seq, const uint8_t *payload, uint16_t len)
{
    if (len != (uint16_t)sizeof(pl_drv8163_start_mon_t))
    {
        send_nack(seq, NACK_BAD_LEN);
        return;
    }

    if (!s_drv8163_ready)
    {
        send_nack(seq, NACK_UNKNOWN);
        return;
    }

    const pl_drv8163_start_mon_t *p = (const pl_drv8163_start_mon_t *)payload;

    s_drv8163.config.current_low_threshold = p->low_th;
    s_drv8163.config.current_high_threshold = p->high_th;
    s_drv8163.config.current_check_interval_ms = p->interval_ms;

    drv8163_motor_state_t st = (p->dir == 0) ? DRV8163_MOTOR_FORWARD : DRV8163_MOTOR_REVERSE;
    (void)drv8163_set_motor_control(&s_drv8163, st, p->speed);

    if (s_drv8163.monitoring_enabled)
    {
        drv8163_stop_current_monitoring(&s_drv8163);
    }

    if (!drv8163_start_current_monitoring(&s_drv8163))
    {
        send_nack(seq, NACK_UNKNOWN);
        return;
    }

    send_ack(seq);
}

static void handle_drv8163_stop(uint16_t seq)
{
    if (!s_drv8163_ready)
    {
        send_nack(seq, NACK_UNKNOWN);
        return;
    }

    drv8163_stop_current_monitoring(&s_drv8163);
    (void)drv8163_set_motor_control(&s_drv8163, DRV8163_MOTOR_STOP, 0u);
    send_ack(seq);
}

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  Existing HX711 handlers (unchanged)                                        */
/* ═══════════════════════════════════════════════════════════════════════════ */

static void handle_hx711_tare(uint16_t seq)
{
    if (!s_hx711_ready)
    {
        send_nack(seq, NACK_UNKNOWN);
        return;
    }

    s_scale_opt.strat = strategy_type_time;
    s_scale_opt.timeout = 1000000; /* 1 second timeout */

    if (scale_zero(&s_scale, &s_scale_opt))
    {
        /* Sync the module-level calibration variable so it stays consistent
         * with s_scale.offset (the canonical tare-adjusted value used by all
         * subsequent scale_weight() and direct-calibration reads). */
        offset = s_scale.offset;
        printf("HX711: Scale zeroed (new offset=%ld)\n", (long)offset);
        send_ack(seq);
    }
    else
    {
        printf("HX711: Scale failed to zero\n");
        send_nack(seq, NACK_UNKNOWN);
    }
}

static void handle_hx711_read(uint16_t seq, const uint8_t *payload, uint16_t len)
{
    if (len != (uint16_t)sizeof(pl_hx711_measure_t))
    {
        send_nack(seq, NACK_BAD_LEN);
        return;
    }

    if (!s_hx711_ready)
    {
        send_nack(seq, NACK_UNKNOWN);
        return;
    }

    const pl_hx711_measure_t *p = (const pl_hx711_measure_t *)payload;

    s_scale_opt.strat = strategy_type_time;
    s_scale_opt.timeout = p->interval_us;

    mass_t mass = {0};
    if (scale_weight(&s_scale, &mass, &s_scale_opt))
    {
#ifdef TUNING
        char str[MASS_TO_STRING_BUFF_SIZE];
        mass_to_string(&mass, str);
        printf("HX711: Weight = %s\n", str);
#endif

        pl_hx711_mass_t response = {
            .mass_ug = (int32_t)mass.ug,
            .unit = (uint8_t)mass.unit,
            ._rsvd = {0, 0, 0},
        };

        (void)uart_send_frame(MSG_HX711_MEASURE, seq, &response, (uint16_t)sizeof(response));
    }
    else
    {
        printf("HX711: Failed to read weight\n");
        send_nack(seq, NACK_UNKNOWN);
    }
}

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  New state-based handlers                                                   */
/* ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief MSG_FLAPS_OPEN (0x40) — drive flap DRV8163 forward; auto-stop on
 *        current drop (open-circuit end-of-travel).
 */
static void handle_flap_open(uint16_t seq)
{
    if (!s_drv8163_ready)
    {
        send_nack(seq, NACK_UNKNOWN);
        return;
    }

    /* Stop any in-progress monitoring before reconfiguring. */
    if (s_drv8163.monitoring_enabled)
    {
        drv8163_stop_current_monitoring(&s_drv8163);
    }

    /* Configure: low threshold = current-drop stop; high threshold disabled. */
    s_drv8163.config.current_low_threshold = (uint16_t)FLAP_OPEN_CURRENT_DROP_TH;
    s_drv8163.config.current_high_threshold = 4095u;
    s_drv8163.config.current_check_interval_ms = (uint32_t)FLAP_MONITOR_INTERVAL_MS;

    (void)drv8163_set_motor_control(&s_drv8163, DRV8163_MOTOR_FORWARD,
                                    (uint16_t)FLAP_OPEN_SPEED_PWM);

    if (!drv8163_start_current_monitoring(&s_drv8163))
    {
        (void)drv8163_set_motor_control(&s_drv8163, DRV8163_MOTOR_STOP, 0u);
        send_nack(seq, NACK_UNKNOWN);
        return;
    }

    /* Start flap 2 if available */
    if (s_drv8163_flap2_ready)
    {
        if (s_drv8163_flap2.monitoring_enabled)
        {
            drv8163_stop_current_monitoring(&s_drv8163_flap2);
        }
        s_drv8163_flap2.config.current_low_threshold = (uint16_t)FLAP_OPEN_CURRENT_DROP_TH;
        s_drv8163_flap2.config.current_high_threshold = 4095u;
        s_drv8163_flap2.config.current_check_interval_ms = (uint32_t)FLAP_MONITOR_INTERVAL_MS;
        (void)drv8163_set_motor_control(&s_drv8163_flap2, DRV8163_MOTOR_FORWARD,
                                        (uint16_t)FLAP_OPEN_SPEED_PWM);
        (void)drv8163_start_current_monitoring(&s_drv8163_flap2);
    }

    s_flap_state = FLAP_SM_OPENING;
    s_flap_start_ms = to_ms_since_boot(get_absolute_time());
    s_flap1_stopped = false;
    s_flap2_stopped = (!s_drv8163_flap2_ready);
    printf("Flap OPEN started\n");
    send_ack(seq);
}

/**
 * @brief MSG_FLAPS_CLOSE (0x41) — drive flap DRV8163 reverse; auto-stop on
 *        torque rise (current high = mechanical stop).
 *        Delegates entirely to flap_close_internal() to avoid code duplication
 *        with the spawn dosing path.
 */
static void handle_flap_close(uint16_t seq)
{
    if (!flap_close_internal())
    {
        send_nack(seq, NACK_UNKNOWN);
        return;
    }
    printf("Flap CLOSE started\n");
    send_ack(seq);
}

/* ── Internal helper: start a stepper motion and arm the pending tracker ───── */

static bool start_stepper_motion(uint8_t dev_idx, int32_t target_steps,
                                 stepper_pending_t *pending,
                                 subsystem_id_t subsys, uint32_t timeout_ms,
                                 uint16_t seq)
{
    int32_t current_steps;
    switch (dev_idx)
    {
    case 0u:
        current_steps = s_arm_pos_steps;
        break;
    case 1u:
        current_steps = s_rack_pos_steps;
        break;
    case 2u:
        current_steps = s_turntable_pos_steps;
        break;
    default:
        send_nack(seq, NACK_UNKNOWN);
        return false;
    }

    /* Cancel any in-flight job on this device. */
    if (s_motion.jobs[dev_idx].active)
    {
        drv8434s_motion_cancel(&s_motion, dev_idx, NULL);
    }
    pending->pending = false;

    int32_t delta = target_steps - current_steps;

    if (delta == 0)
    {
        /* Already at target — ACK and report done immediately. */
        send_ack(seq);
        send_motion_done(subsys, MOTION_OK, current_steps);
        return true;
    }

    if (!drv8434s_motion_start(&s_motion, dev_idx, delta, 0u))
    {
        send_nack(seq, NACK_UNKNOWN);
        return false;
    }

    if (!ensure_step_timer_running())
    {
        drv8434s_motion_cancel(&s_motion, dev_idx, NULL);
        send_nack(seq, NACK_UNKNOWN);
        return false;
    }

    pending->pending = true;
    pending->target_steps = target_steps;
    pending->subsys = subsys;
    pending->start_ms = to_ms_since_boot(get_absolute_time());
    pending->timeout_ms = timeout_ms;
    pending->dev_idx = dev_idx;

    send_ack(seq);
    return true;
}

/**
 * @brief MSG_ARM_MOVE (0x42) — move arm stepper (device 0) to named position.
 */
static void handle_arm_move(uint16_t seq, const uint8_t *payload, uint16_t len)
{
    if (len < (uint16_t)sizeof(pl_arm_move_t))
    {
        send_nack(seq, NACK_BAD_LEN);
        return;
    }

    if (!s_stepper_ready)
    {
        send_nack(seq, NACK_UNKNOWN);
        return;
    }

    const pl_arm_move_t *p = (const pl_arm_move_t *)payload;
    int32_t target;

    switch ((arm_pos_t)p->position)
    {
    case ARM_POS_PRESS:
        target = (int32_t)ARM_STEPS_PRESS;
        break;
    case ARM_POS_1:
        target = (int32_t)ARM_STEPS_POS1;
        break;
    case ARM_POS_2:
        target = (int32_t)ARM_STEPS_POS2;
        break;
    default:
        send_nack(seq, NACK_UNKNOWN);
        return;
    }

    printf("Arm move → %li steps\n", (long)target);
    (void)start_stepper_motion(0u, target, &s_arm_pending,
                               SUBSYS_ARM, (uint32_t)ARM_MOTION_TIMEOUT_MS, seq);
}

/**
 * @brief MSG_RACK_MOVE (0x43) — move rack stepper (device 1) to named position.
 *        RACK_POS_HOME drives the rack to absolute step 0.
 */
static void handle_rack_move(uint16_t seq, const uint8_t *payload, uint16_t len)
{
    if (len < (uint16_t)sizeof(pl_rack_move_t))
    {
        send_nack(seq, NACK_BAD_LEN);
        return;
    }

    if (!s_stepper_ready)
    {
        send_nack(seq, NACK_UNKNOWN);
        return;
    }

    const pl_rack_move_t *p = (const pl_rack_move_t *)payload;
    int32_t target;

    switch ((rack_pos_t)p->position)
    {
    case RACK_POS_HOME:
        target = 0;
        break;
    case RACK_POS_EXTEND:
        target = (int32_t)RACK_STEPS_EXTEND;
        break;
    case RACK_POS_PRESS:
        target = (int32_t)RACK_STEPS_PRESS;
        break;
    default:
        send_nack(seq, NACK_UNKNOWN);
        return;
    }

    printf("Rack move → %li steps\n", (long)target);
    (void)start_stepper_motion(1u, target, &s_rack_pending,
                               SUBSYS_RACK, (uint32_t)RACK_MOTION_TIMEOUT_MS, seq);
}

/**
 * @brief MSG_TURNTABLE_GOTO (0x44) — move turntable stepper (device 2) to
 *        named angular position.  NACKs if turntable is not yet homed.
 */
static void handle_turntable_goto(uint16_t seq, const uint8_t *payload, uint16_t len)
{
    if (len < (uint16_t)sizeof(pl_turntable_goto_t))
    {
        send_nack(seq, NACK_BAD_LEN);
        return;
    }

    if (!s_stepper_ready)
    {
        send_nack(seq, NACK_UNKNOWN);
        return;
    }

    if (!s_turntable_homed)
    {
        /* Turntable must be homed before any GOTO command. */
        printf("Turntable GOTO rejected: not homed\n");
        send_nack(seq, NACK_UNKNOWN);
        return;
    }

    const pl_turntable_goto_t *p = (const pl_turntable_goto_t *)payload;
    int32_t target;

    switch ((turntable_pos_t)p->position)
    {
    case TURNTABLE_POS_A:
        target = (int32_t)TURNTABLE_STEPS_A;
        break;
    case TURNTABLE_POS_B:
        target = (int32_t)TURNTABLE_STEPS_B;
        break;
    case TURNTABLE_POS_C:
        target = (int32_t)TURNTABLE_STEPS_C;
        break;
    case TURNTABLE_POS_D:
        target = (int32_t)TURNTABLE_STEPS_D;
        break;
    default:
        send_nack(seq, NACK_UNKNOWN);
        return;
    }

    printf("Turntable GOTO → %li steps\n", (long)target);
    (void)start_stepper_motion(2u, target, &s_turntable_pending,
                               SUBSYS_TURNTABLE,
                               (uint32_t)TURNTABLE_MOTION_TIMEOUT_MS, seq);
}

/**
 * @brief MSG_TURNTABLE_HOME (0x45) — zero the turntable position counter and
 *        mark it as calibrated.  No physical motion is performed here; this
 *        command is used to declare the current position as the reference zero.
 */
static void handle_turntable_home(uint16_t seq)
{
    /* Cancel any in-progress turntable motion. */
    if (s_motion.jobs[2].active)
    {
        drv8434s_motion_cancel(&s_motion, 2u, NULL);
        s_turntable_pending.pending = false;
    }

    s_turntable_pos_steps = 0;
    s_turntable_homed = true;
    printf("Turntable HOME: position zeroed, homed=true\n");
    send_ack(seq);
}

/**
 * @brief MSG_HOTWIRE_SET (0x46) — enable or disable the hot-wire DRV8163.
 *
 * Safety interlock: if vacuum2 is currently ON (using the REVERSE channel of
 * the same H-bridge), it is stopped first before enabling the hot wire.
 */
static void handle_hotwire_set(uint16_t seq, const uint8_t *payload, uint16_t len)
{
    if (len < (uint16_t)sizeof(pl_hotwire_set_t))
    {
        send_nack(seq, NACK_BAD_LEN);
        return;
    }

    if (!s_drv8163_hotwire_ready)
    {
        send_nack(seq, NACK_UNKNOWN);
        return;
    }

    const pl_hotwire_set_t *p = (const pl_hotwire_set_t *)payload;

    if (p->enable)
    {
        /* Stop vacuum2 first if it is using the REVERSE channel of this H-bridge. */
        if (s_vacuum2_on)
        {
            (void)drv8163_set_motor_control(&s_drv8163_hotwire, DRV8163_MOTOR_STOP, 0u);
            s_vacuum2_on = false;
            printf("Hotwire ON: stopped vacuum2 (mutual exclusion)\n");
        }
        /* Forward direction at constant current duty cycle. */
        (void)drv8163_set_motor_control(&s_drv8163_hotwire,
                                        DRV8163_MOTOR_FORWARD,
                                        (uint16_t)HOTWIRE_CURRENT_DUTY);
        printf("Hotwire ON (duty=%u)\n", (unsigned)HOTWIRE_CURRENT_DUTY);
    }
    else
    {
        /* Coast the wire (both outputs low). */
        (void)drv8163_set_motor_control(&s_drv8163_hotwire, DRV8163_MOTOR_STOP, 0u);
        printf("Hotwire OFF\n");
    }

    send_ack(seq);
}

/**
 * @brief MSG_VACUUM_SET (0x47) — assert or de-assert the vacuum pump trigger.
 */
static void handle_vacuum_set(uint16_t seq, const uint8_t *payload, uint16_t len)
{
    if (len < (uint16_t)sizeof(pl_vacuum_set_t))
    {
        send_nack(seq, NACK_BAD_LEN);
        return;
    }

    const pl_vacuum_set_t *p = (const pl_vacuum_set_t *)payload;

    s_vacuum_on = (bool)p->enable;
    gpio_put((uint)VACUUM_TRIGGER_PIN, s_vacuum_on ? 1u : 0u);

    if (!s_vacuum_on)
    {
        /* Pump stopped — clear status and notify ESP32 immediately. */
        s_vacuum_status = VACUUM_OFF;
        send_vacuum_status(VACUUM_OFF, 0u);
        printf("Vacuum OFF\n");
    }
    else
    {
        /* Reset sampling timestamps so the first sample fires promptly. */
        s_vacuum_last_sample_ms = to_ms_since_boot(get_absolute_time());
        s_vacuum_last_status_ms = s_vacuum_last_sample_ms;
        /* Reset pulse counter atomically. */
        uint32_t saved = save_and_disable_interrupts();
        s_vacuum_pulse_count = 0u;
        restore_interrupts(saved);
        printf("Vacuum ON\n");
    }

    send_ack(seq);
}

/**
 * @brief MSG_VACUUM2_SET (0x48) — enable or disable the second vacuum pump.
 *
 * The second vacuum pump is driven via the REVERSE channel (IN2) of the
 * hotwire DRV8163.  It is mutually exclusive with the hot wire (FORWARD
 * channel / IN1).  Enabling vacuum2 always stops the hot wire first.
 */
static void handle_vacuum2_set(uint16_t seq, const uint8_t *payload, uint16_t len)
{
    if (len < (uint16_t)sizeof(pl_vacuum2_set_t))
    {
        send_nack(seq, NACK_BAD_LEN);
        return;
    }

    if (!s_drv8163_hotwire_ready)
    {
        send_nack(seq, NACK_UNKNOWN);
        return;
    }

    const pl_vacuum2_set_t *pl = (const pl_vacuum2_set_t *)payload;

    if (pl->enable)
    {
        /* Stop hotwire first (safety — mutually exclusive with vacuum2). */
        (void)drv8163_set_motor_control(&s_drv8163_hotwire, DRV8163_MOTOR_STOP, 0u);
        /* Drive IN2 high via REVERSE mode at full duty to trigger the pump. */
        (void)drv8163_set_motor_control(&s_drv8163_hotwire, DRV8163_MOTOR_REVERSE, 4095u);
        s_vacuum2_on = true;
        printf("Vacuum2 ON (hotwire DRV8163 REVERSE channel)\n");
    }
    else
    {
        (void)drv8163_set_motor_control(&s_drv8163_hotwire, DRV8163_MOTOR_STOP, 0u);
        s_vacuum2_on = false;
        printf("Vacuum2 OFF\n");
    }

    send_ack(seq);
}

/**
 * @brief MSG_DISPENSE_SPAWN (0x49) - run closed loop dispensing procedure.
 *
 * The flaps should start closed, while the bag should be weighed and opned
 * The flaps will begin to open and a innoculation rate is targeted on the way
 * to a target weight, calculated by percentage of bag weight.
 */
static void handle_dispense_spawn(uint16_t seq, const uint8_t *payload, uint16_t len)
{
    typedef struct __attribute__((packed))
    {
        uint16_t bag_mass;
        uint16_t spawn_mass;
        uint16_t innoc_percent;
        uint8_t bag_number;
    } pl_innoculate_bag_legacy_t;

    if (len != (uint16_t)sizeof(pl_innoculate_bag_t) &&
        len != (uint16_t)sizeof(pl_innoculate_bag_legacy_t))
    {
        send_nack(seq, NACK_BAD_LEN);
        return;
    }
    if (!s_hx711_ready)
    {
        send_nack(seq, NACK_UNKNOWN);
        return;
    }

    if (s_spawn.active)
    {
        /* Stop any prior dosing run */
        spawn_stop_timer();
        spawn_close_flaps();
        s_spawn.active = false;
    }

    const pl_innoculate_bag_t *pl = (const pl_innoculate_bag_t *)payload;
    uint16_t innoc_percent = 0u;
    uint8_t dose_style = DOSE_STYLE_A;
    uint8_t bag_number = 0u;

    if (len == (uint16_t)sizeof(pl_innoculate_bag_legacy_t))
    {
        const pl_innoculate_bag_legacy_t *legacy = (const pl_innoculate_bag_legacy_t *)payload;
        innoc_percent = legacy->innoc_percent;
        bag_number = legacy->bag_number;
    }
    else
    {
        innoc_percent = pl->innoc_percent;
        bag_number = pl->bag_number;
        dose_style = (pl->dose_style == DOSE_STYLE_B) ? DOSE_STYLE_B : DOSE_STYLE_A;
    }

    if (innoc_percent < SPAWN_INNOC_PCT_MIN_X10 || innoc_percent > SPAWN_INNOC_PCT_MAX_X10)
    {
        send_nack(seq, NACK_UNKNOWN);
        return;
    }

    /* Drain stale HX711 FIFO samples before taking the baseline.
     * The HX711 runs at 80 SPS continuously; between doses the FIFO/queue
     * accumulates readings from the previous weight.  Reading and discarding
     * 25 samples (~312 ms of fresh HX711 output) flushes the backlog before
     * the actual baseline measurement is taken. */
    {
        mass_t discard_mass = {0};
        int32_t discard_buf[25];
        scale_options_t discard_opt;
        scale_options_get_default(&discard_opt);
        discard_opt.strat = strategy_type_samples;
        discard_opt.samples = 25u;
        discard_opt.buffer = discard_buf;
        discard_opt.bufflen = 25u;
        (void)scale_weight(&s_scale, &discard_mass, &discard_opt); /* result discarded */
    }

    /* Take a fresh baseline — FIFO is now clear of stale data.
     * 10-element buffer averages the last 10 of ~24 samples taken in 300 ms. */
    mass_t mass = {0};
    int32_t baseline_buf[10];
    scale_options_t baseline_opt;
    scale_options_get_default(&baseline_opt);
    baseline_opt.strat = strategy_type_time;
    baseline_opt.timeout = 300000; /* 300 ms */
    baseline_opt.buffer = baseline_buf;
    baseline_opt.bufflen = 10u;
    if (!scale_weight(&s_scale, &mass, &baseline_opt))
    {
        send_nack(seq, NACK_UNKNOWN);
        return;
    }

    memset(&s_spawn, 0, sizeof(s_spawn));
    s_spawn.active = true;
    s_spawn.agitating = false;
    s_spawn.retry = 0u;
    s_spawn.max_retries = SPAWN_MAX_RETRIES;
    s_spawn.bag_number = bag_number;
    s_spawn.start_mass_ug = (uint32_t)mass.ug;
    double m;
    mass_get_value(&mass, &m);
    printf("Initial mass %f\n", m);
    /* innoc_percent is x10 percent (e.g., 250 = 25.0%) */
    uint64_t target_ug = ((uint64_t)s_spawn.start_mass_ug * (uint64_t)innoc_percent) / 1000ULL;
    s_spawn.target_ug = (uint32_t)target_ug;
    s_spawn.dispensed_ug = 0u;
    s_spawn.window_start_ug = 0u; /* start of first 500 ms flow window */
    s_spawn.last_tick_ug = 0u;    /* previous-tick baseline for nudge delta */
    s_spawn.spawn_remaining_ug = ((uint32_t)pl->spawn_mass) * 1000000UL;
    s_spawn.last_flow_check = get_absolute_time();
    s_spawn.nudging = false;
    s_spawn.startup_opening = true; /* drive open until first weight change before closed loop */

    /* Drive flap open at full PWM and arm undercurrent monitoring.
     * If the flap reaches end-of-travel (no load = undercurrent) before any
     * weight change is registered, the startup_opening check treats that as a
     * flow failure.  Monitoring is stopped once flow is confirmed. */
    if (s_drv8163.monitoring_enabled)
    {
        drv8163_stop_current_monitoring(&s_drv8163);
    }
    s_drv8163.config.current_low_threshold = (uint16_t)FLAP_OPEN_CURRENT_DROP_TH;
    s_drv8163.config.current_high_threshold = 4095u;
    s_drv8163.config.current_check_interval_ms = (uint32_t)FLAP_MONITOR_INTERVAL_MS;
    spawn_set_flap_pwm(true, (uint16_t)SPAWN_OPEN_PWM);
    (void)drv8163_start_current_monitoring(&s_drv8163);

    if (s_drv8163_flap2_ready)
    {
        if (s_drv8163_flap2.monitoring_enabled)
        {
            drv8163_stop_current_monitoring(&s_drv8163_flap2);
        }
        s_drv8163_flap2.config.current_low_threshold = (uint16_t)FLAP_OPEN_CURRENT_DROP_TH;
        s_drv8163_flap2.config.current_high_threshold = 4095u;
        s_drv8163_flap2.config.current_check_interval_ms = (uint32_t)FLAP_MONITOR_INTERVAL_MS;
        (void)drv8163_set_motor_control(&s_drv8163_flap2, DRV8163_MOTOR_FORWARD,
                                        (uint16_t)SPAWN_OPEN_PWM);
        (void)drv8163_start_current_monitoring(&s_drv8163_flap2);
    }

    if (!add_repeating_timer_ms(SPAWN_TIMER_PERIOD_MS, dispense_spawn_callback, NULL, &s_spawn.timer))
    {
        spawn_close_flaps();
        s_spawn.active = false;
        send_nack(seq, NACK_UNKNOWN);
        return;
    }

    printf("Spawn: dose style %c\n", (dose_style == DOSE_STYLE_B) ? 'B' : 'A');
    send_ack(seq);
    send_spawn_status(SPAWN_STATUS_RUNNING);
}

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  Frame dispatch                                                              */
/* ═══════════════════════════════════════════════════════════════════════════ */

static void dispatch_decoded(const uint8_t *decoded, size_t decoded_len)
{
    if (decoded_len < sizeof(proto_hdr_t) + 2u)
    {
        return;
    }
    printf("message received\n");
    proto_hdr_t hdr;
    memcpy(&hdr, decoded, sizeof(hdr));

    const uint16_t got_crc = (uint16_t)decoded[decoded_len - 2u] |
                             ((uint16_t)decoded[decoded_len - 1u] << 8);
    const uint16_t calc_crc = proto_crc16_ccitt(decoded, (uint32_t)(decoded_len - 2u));

    if (got_crc != calc_crc)
    {
        send_nack(hdr.seq, NACK_BAD_CRC);
        return;
    }

    if (hdr.version != PROTO_VERSION)
    {
        send_nack(hdr.seq, NACK_BAD_VERSION);
        return;
    }

    if (hdr.len > PROTO_MAX_PAYLOAD)
    {
        send_nack(hdr.seq, NACK_BAD_LEN);
        return;
    }

    if (decoded_len != (size_t)sizeof(proto_hdr_t) + hdr.len + 2u)
    {
        send_nack(hdr.seq, NACK_BAD_LEN);
        return;
    }

    const uint8_t *payload = decoded + sizeof(proto_hdr_t);

    switch ((msg_type_t)hdr.type)
    {
        /* ── Existing handlers ──────────────────────────────────────────────── */

    case MSG_PING:
        printf("Ping\n");
        send_ack(hdr.seq);
        break;

    case MSG_MOTOR_DRV8163_START_MON:
        printf("DRV8163 Start Mon\n");
        handle_drv8163_start(hdr.seq, payload, hdr.len);
        break;

    case MSG_MOTOR_DRV8163_STOP_MON:
        printf("DRV8163 Stop Mon\n");
        handle_drv8163_stop(hdr.seq);
        break;

    case MSG_MOTOR_STEPPER_ENABLE:
        printf("Stepper Enable\n");
        handle_stepper_enable(hdr.seq, payload, hdr.len);
        break;

    case MSG_MOTOR_STEPPER_STEPJOB:
        printf("Stepper StepJob\n");
        handle_stepper_stepjob(hdr.seq, payload, hdr.len);
        break;

    case MSG_HX711_TARE:
        printf("HX711 Tare\n");
        handle_hx711_tare(hdr.seq);
        break;

    case MSG_HX711_MEASURE:
        printf("HX711 Measure\n");
        handle_hx711_read(hdr.seq, payload, hdr.len);
        break;

        /* ── New state-based handlers ───────────────────────────────────────── */

    case MSG_FLAPS_OPEN:
        printf("Flaps Open\n");
        handle_flap_open(hdr.seq);
        break;

    case MSG_FLAPS_CLOSE:
        printf("Flaps Close\n");
        handle_flap_close(hdr.seq);
        break;

    case MSG_ARM_MOVE:
        printf("Arm Move\n");
        handle_arm_move(hdr.seq, payload, hdr.len);
        break;

    case MSG_RACK_MOVE:
        printf("Rack Move\n");
        handle_rack_move(hdr.seq, payload, hdr.len);
        break;

    case MSG_TURNTABLE_GOTO:
        printf("Turntable Goto\n");
        handle_turntable_goto(hdr.seq, payload, hdr.len);
        break;

    case MSG_TURNTABLE_HOME:
        printf("Turntable Home\n");
        handle_turntable_home(hdr.seq);
        break;

    case MSG_HOTWIRE_SET:
        printf("Hotwire Set\n");
        handle_hotwire_set(hdr.seq, payload, hdr.len);
        break;

    case MSG_VACUUM_SET:
        printf("Vacuum Set\n");
        handle_vacuum_set(hdr.seq, payload, hdr.len);
        break;

    case MSG_VACUUM2_SET:
        printf("Vacuum2 Set\n");
        handle_vacuum2_set(hdr.seq, payload, hdr.len);
        break;

    case MSG_DISPENSE_SPAWN:
        printf("Innoculating\n");
        handle_dispense_spawn(hdr.seq, payload, hdr.len);
        break;

    case MSG_CTRL_STOP:
        printf("Abort\n");
        if (s_spawn.active)
        {
            spawn_stop_timer();
            spawn_close_flaps();
            s_spawn.active = false;
            s_spawn.agitating = false;
            send_spawn_status(SPAWN_STATUS_ABORTED);
        }
        send_ack(hdr.seq);
        break;

    default:
        send_nack(hdr.seq, NACK_UNKNOWN);
        break;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  COBS frame processing                                                       */
/* ═══════════════════════════════════════════════════════════════════════════ */

static void process_frame(void)
{
    uint8_t decoded[UART_DECODED_FRAME_MAX];

    if (s_frame_len == 0u || s_frame_overflow)
    {
        s_frame_len = 0u;
        s_frame_overflow = false;
        return;
    }

    const size_t decoded_len = cobs_decode(s_frame_buf, s_frame_len, decoded);
    if (decoded_len == 0u || decoded_len > sizeof(decoded))
    {
        s_frame_len = 0u;
        return;
    }

    dispatch_decoded(decoded, decoded_len);
    s_frame_len = 0u;
}

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  Subsystem init helpers                                                      */
/* ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Initialise the second flap DRV8163 instance.
 *
 * Uses FLAP2_CTRL_A_PIN / FLAP2_CTRL_B_PIN / FLAP2_ADC_SENSE_PIN /
 * FLAP2_ADC_CHANNEL from board_pins.h.  Shares all threshold constants
 * with the primary flap instance.
 */
static void init_drv8163_flap2(void)
{
    s_drv8163_flap2_ready = false;

    drv8163_config_t cfg = {
        .user_ctx = NULL,
        .ctrl_a_pin = (uint8_t)FLAP2_CTRL_A_PIN,
        .ctrl_b_pin = (uint8_t)FLAP2_CTRL_B_PIN,
        .sense_pin = (uint8_t)FLAP2_ADC_SENSE_PIN,
        .sense_adc_channel = (uint8_t)FLAP2_ADC_CHANNEL,
        .current_high_threshold = (uint16_t)DRV8163_DEFAULT_HIGH_TH,
        .current_low_threshold = (uint16_t)DRV8163_DEFAULT_LOW_TH,
        .pwm_frequency_hz = (uint32_t)DRV8163_DEFAULT_PWM_HZ,
        .current_check_interval_ms = (uint32_t)FLAP_MONITOR_INTERVAL_MS,
        .startup_blanking_ms = (uint32_t)DRV8163_DEFAULT_STARTUP_BLANKING_MS,
        .current_cb = NULL,
        .delay_ms = NULL,
    };

    if (!drv8163_init(&s_drv8163_flap2, &cfg))
    {
        printf("uart_server: DRV8163 flap2 init failed\n");
        return;
    }

    s_drv8163_flap2_ready = true;
    printf("uart_server: DRV8163 flap2 ready (A=%d B=%d ADC_CH=%d)\n",
           FLAP2_CTRL_A_PIN, FLAP2_CTRL_B_PIN, FLAP2_ADC_CHANNEL);
}

static void init_drv8163(void)
{
    s_drv8163_ready = false;

    if (DRV8163_CTRL_A_GPIO < 0 || DRV8163_CTRL_B_GPIO < 0 || DRV8163_SENSE_GPIO < 0 ||
        DRV8163_SENSE_ADC_CH < 0 || DRV8163_SENSE_ADC_CH > 3)
    {
        printf("uart_server: DRV8163 pins/ch not set in board_pins.h; motor RPC disabled\n");
        return;
    }

    drv8163_config_t cfg = {
        .user_ctx = NULL,
        .ctrl_a_pin = (uint8_t)DRV8163_CTRL_A_GPIO,
        .ctrl_b_pin = (uint8_t)DRV8163_CTRL_B_GPIO,
        .sense_pin = (uint8_t)DRV8163_SENSE_GPIO,
        .sense_adc_channel = (uint8_t)DRV8163_SENSE_ADC_CH,
        .current_high_threshold = DRV8163_DEFAULT_HIGH_TH,
        .current_low_threshold = DRV8163_DEFAULT_LOW_TH,
        .pwm_frequency_hz = DRV8163_DEFAULT_PWM_HZ,
        .current_check_interval_ms = DRV8163_DEFAULT_CHECK_INTERVAL_MS,
        .startup_blanking_ms = DRV8163_DEFAULT_STARTUP_BLANKING_MS,
        .current_cb = NULL,
        .delay_ms = NULL,
    };

    if (!drv8163_init(&s_drv8163, &cfg))
    {
        printf("uart_server: DRV8163 init failed\n");
        return;
    }

    s_drv8163_ready = true;
    printf("uart_server: DRV8163 flap1 ready (A=%d B=%d ADC_CH=%d)\n",
           DRV8163_CTRL_A_GPIO, DRV8163_CTRL_B_GPIO, DRV8163_SENSE_ADC_CH);
}

/**
 * @brief Initialise the hot-wire DRV8163 instance (secondary H-bridge).
 *
 * Hot wire is unidirectional (forward only, constant current duty cycle).
 * The current monitoring thresholds are set wide — we do not use auto-stop
 * for the hot wire; ON/OFF is controlled entirely by MSG_HOTWIRE_SET.
 */
static void init_drv8163_hotwire(void)
{
    s_drv8163_hotwire_ready = false;

    drv8163_config_t cfg = {
        .user_ctx = NULL,
        .ctrl_a_pin = (uint8_t)HOTWIRE_PIN_IN1,
        .ctrl_b_pin = (uint8_t)HOTWIRE_PIN_IN2,
        .sense_pin = (uint8_t)HOTWIRE_ADC_SENSE_PIN,
        .sense_adc_channel = (uint8_t)HOTWIRE_ADC_CHANNEL,
        .current_high_threshold = 4095u, /* never auto-stop */
        .current_low_threshold = 0u,     /* never auto-stop */
        .pwm_frequency_hz = 20000u,
        .current_check_interval_ms = (uint32_t)HOTWIRE_MONITOR_INTERVAL_MS,
        .startup_blanking_ms = (uint32_t)DRV8163_DEFAULT_STARTUP_BLANKING_MS,
        .current_cb = NULL,
        .delay_ms = NULL,
    };

    if (!drv8163_init(&s_drv8163_hotwire, &cfg))
    {
        printf("uart_server: DRV8163 hotwire init failed\n");
        return;
    }

    s_drv8163_hotwire_ready = true;
    printf("uart_server: DRV8163 hotwire ready (IN1=%d IN2=%d ADC_CH=%d)\n",
           HOTWIRE_PIN_IN1, HOTWIRE_PIN_IN2, HOTWIRE_ADC_CHANNEL);
}

static void init_drv8434s_chain(void)
{
    s_stepper_ready = false;

#if DRV8434S_N_DEVICES == 0
    printf("uart_server: DRV8434S_N_DEVICES=0; stepper RPC disabled\n");
    return;
#else
    if (DRV8434S_SCK_GPIO < 0 || DRV8434S_MOSI_GPIO < 0 ||
        DRV8434S_MISO_GPIO < 0 || DRV8434S_CS_GPIO < 0)
    {
        printf("uart_server: DRV8434S SPI pins not set in board_pins.h; stepper RPC disabled\n");
        return;
    }

    /* Initialise SPI hardware.
     * DRV8434S requires SPI Mode 1 (CPOL=0, CPHA=1), MSB-first, 8-bit frames. */
    spi_inst_t *spi = (DRV8434S_SPI_ID == 0) ? spi0 : spi1;
    spi_init(spi, (uint)DRV8434S_SPI_BAUD);
    spi_set_format(spi, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);

    gpio_set_function((uint)DRV8434S_SCK_GPIO, GPIO_FUNC_SPI);
    gpio_set_function((uint)DRV8434S_MOSI_GPIO, GPIO_FUNC_SPI);
    gpio_set_function((uint)DRV8434S_MISO_GPIO, GPIO_FUNC_SPI);

    /* CS is software-controlled (active-low); start deasserted (high). */
    gpio_init((uint)DRV8434S_CS_GPIO);
    gpio_set_dir((uint)DRV8434S_CS_GPIO, GPIO_OUT);
    gpio_put((uint)DRV8434S_CS_GPIO, 1u);

    drv8434s_chain_config_t cfg = {
        .user_ctx = spi,
        .spi_xfer = stepper_spi_xfer,
        .cs = stepper_cs,
        .delay_ms = stepper_delay_ms,
        .delay_us = stepper_delay_us,
        .n_devices = (uint8_t)DRV8434S_N_DEVICES,
    };

    if (!drv8434s_chain_init(&s_stepper_chain, &cfg))
    {
        printf("uart_server: DRV8434S chain init failed\n");
        return;
    }

    /* Enable SPI step/direction mode for all devices. */
    for (uint8_t k = 0; k < s_stepper_chain.cfg.n_devices; ++k)
    {
        if (!drv8434s_chain_set_spi_step_mode(&s_stepper_chain, k, NULL))
        {
            printf("uart_server: DRV8434S set SPI step mode failed (dev %u)\n", k);
            return;
        }
    }

    /* Enable motor outputs on all devices (sets EN_OUT in CTRL2).
     * The DRV8434S powers up with EN_OUT=0 (tristate).  Without this,
     * STEP pulses are processed internally but no current flows to the coils. */
    for (uint8_t k = 0; k < s_stepper_chain.cfg.n_devices; ++k)
    {
        if (!drv8434s_chain_enable(&s_stepper_chain, k, NULL))
        {
            printf("uart_server: DRV8434S enable outputs failed (dev %u)\n", k);
            return;
        }
    }

    printf("uart_server: DRV8434S all %u device(s) enabled (EN_OUT=1)\n",
           (unsigned)s_stepper_chain.cfg.n_devices);

    /* Initialise the non-blocking motion engine.
     * torque_sample_div=10 → sample torque every 10th tick. */
    if (!drv8434s_motion_init(&s_motion, &s_stepper_chain,
                              stepper_motion_done, NULL, 10u))
    {
        printf("uart_server: DRV8434S motion engine init failed\n");
        return;
    }

    s_stepper_ready = true;
    printf("uart_server: DRV8434S chain ready (SPI%u, %d device(s), "
           "SCK=%d MOSI=%d MISO=%d CS=%d)\n",
           (unsigned)DRV8434S_SPI_ID,
           DRV8434S_N_DEVICES,
           DRV8434S_SCK_GPIO, DRV8434S_MOSI_GPIO,
           DRV8434S_MISO_GPIO, DRV8434S_CS_GPIO);
#endif
}

static void init_hx711(void)
{
    s_hx711_ready = false;

    if (HX711_CLK_GPIO < 0 || HX711_DATA_GPIO < 0)
    {
        printf("uart_server: HX711 pins not set in board_pins.h; HX711 RPC disabled\n");
        return;
    }

    scale_options_get_default(&s_scale_opt);
    s_scale_opt.buffer = s_scale_valbuff;
    s_scale_opt.bufflen = sizeof(s_scale_valbuff) / sizeof(s_scale_valbuff[0]);

    hx711_config_t hxcfg = {0};
    hx711_get_default_config(&hxcfg);
    hxcfg.clock_pin = (uint)HX711_CLK_GPIO;
    hxcfg.data_pin = (uint)HX711_DATA_GPIO;

    hx711_init(&s_hx711, &hxcfg);
    hx711_power_up(&s_hx711, hx711_gain_128);
    hx711_wait_settle(hx711_rate_80);

    if (!hx711_scale_adaptor_init(&s_hxsa, &s_hx711))
    {
        printf("uart_server: HX711 scale adaptor init failed\n");
        return;
    }

    scale_init(&s_scale,
               hx711_scale_adaptor_get_base(&s_hxsa),
               unit,
               refUnit,
               offset);

    s_hx711_ready = true;
    printf("uart_server: HX711 initialized (CLK=%d DATA=%d)\n",
           HX711_CLK_GPIO, HX711_DATA_GPIO);
}

/**
 * @brief Initialise the vacuum pump GPIO and RPM sense interrupt.
 */
static void init_vacuum(void)
{
    /* Trigger output — pump is off at boot. */
    gpio_init((uint)VACUUM_TRIGGER_PIN);
    gpio_set_dir((uint)VACUUM_TRIGGER_PIN, GPIO_OUT);
    gpio_put((uint)VACUUM_TRIGGER_PIN, 0u);

    /* RPM sense input with rising-edge interrupt. */
    gpio_init((uint)VACUUM_RPM_SENSE_PIN);
    gpio_set_dir((uint)VACUUM_RPM_SENSE_PIN, GPIO_IN);
    gpio_pull_down((uint)VACUUM_RPM_SENSE_PIN);
    gpio_set_irq_enabled_with_callback((uint)VACUUM_RPM_SENSE_PIN,
                                       GPIO_IRQ_EDGE_RISE, true,
                                       vacuum_rpm_isr);

    s_vacuum_on = false;
    s_vacuum_pulse_count = 0u;
    s_vacuum_last_sample_ms = 0u;
    s_vacuum_last_status_ms = 0u;
    s_vacuum_status = VACUUM_OFF;

    printf("uart_server: Vacuum pump ready (TRIGGER=%d RPM_SENSE=%d)\n",
           VACUUM_TRIGGER_PIN, VACUUM_RPM_SENSE_PIN);
}

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  Public API                                                                  */
/* ═══════════════════════════════════════════════════════════════════════════ */

void uart_server_init(void)
{
    memset(&s_rx_ring, 0, sizeof(s_rx_ring));
    s_frame_len = 0u;
    s_frame_overflow = false;

    memset(&s_arm_pending, 0, sizeof(s_arm_pending));
    memset(&s_rack_pending, 0, sizeof(s_rack_pending));
    memset(&s_turntable_pending, 0, sizeof(s_turntable_pending));

    s_flap_state = FLAP_SM_IDLE;
    s_arm_pos_steps = 0;
    s_rack_pos_steps = 0;
    s_turntable_pos_steps = 0;
    /* Auto-home turntable at boot: declare current physical position as step 0,
     * mirroring how s_arm_pos_steps and s_rack_pos_steps are initialised.
     * MSG_TURNTABLE_HOME can still be sent later to re-zero if needed. */
    s_turntable_homed = true;

    s_uart = (PICO_UART_ID == 0) ? uart0 : uart1;
    s_uart_ready = false;

    if (PICO_UART_TX_GPIO < 0 || PICO_UART_RX_GPIO < 0)
    {
        printf("uart_server: UART pins unset in board_pins.h; UART disabled\n");
    }
    else
    {
        uart_init(s_uart, PICO_UART_BAUD);
        gpio_set_function((uint)PICO_UART_TX_GPIO, GPIO_FUNC_UART);
        gpio_set_function((uint)PICO_UART_RX_GPIO, GPIO_FUNC_UART);
        s_uart_ready = true;

        printf("uart_server: UART%u up @ %u baud (TX=%d RX=%d)\n",
               (unsigned)PICO_UART_ID,
               (unsigned)PICO_UART_BAUD,
               PICO_UART_TX_GPIO,
               PICO_UART_RX_GPIO);
    }

    /* Existing subsystems */
    init_drv8163();
    init_drv8163_flap2();
    init_hx711();
    init_drv8434s_chain();

    /* New subsystems */
    init_drv8163_hotwire();
    init_vacuum();
}

void uart_server_poll(void)
{
    if (!s_uart_ready)
    {
        return;
    }

    /* ── Run state-machine ticks before processing incoming bytes ── */
    vacuum_sm_tick();
    flap_sm_tick();
    stepper_completion_tick();

    /* ── Drain hardware UART FIFO into the ring buffer ─────────── */
    while (uart_is_readable(s_uart))
    {
        uint8_t b = uart_getc(s_uart);
        if (!rx_ring_push(b))
        {
            s_frame_len = 0u;
            s_frame_overflow = true;
        }
    }

    /* ── Process bytes from the ring buffer ─────────────────────── */
    uint8_t b = 0u;
    while (rx_ring_pop(&b))
    {
        if (b == PROTO_DELIM)
        {
            process_frame();
            continue;
        }

        if (s_frame_overflow)
        {
            continue;
        }

        if (s_frame_len >= sizeof(s_frame_buf))
        {
            s_frame_overflow = true;
            continue;
        }

        s_frame_buf[s_frame_len++] = b;
    }
}
