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
#include "drivers/drv8263/drv8263.h"
#include "drivers/drv8434s/drv8434s.h"

#include "extern/pico-scale/include/hx711_scale_adaptor.h"
#include "extern/pico-scale/include/scale.h"
#include "extern/pico-scale/extern/hx711-pico-c/include/common.h"

#include "shared/proto/proto.h"
#include "shared/proto/cobs.h"

#ifndef STEPPER_SOFT_TORQUE_LIMIT
#define STEPPER_SOFT_TORQUE_LIMIT PROTO_STEPPER_SOFT_TORQUE_LIMIT_DEFAULT
#endif

/* ── Frame-level constants ─────────────────────────────────────────────────── */

#define UART_RX_RING_SIZE 512u
#define UART_ENCODED_FRAME_MAX 256u
#define UART_DECODED_FRAME_MAX ((size_t)sizeof(proto_hdr_t) + PROTO_MAX_PAYLOAD + 2u)
#define VACUUM_RPM_TIMEOUT_MS 500u
#define VACUUM_RPM_DEBOUNCE_US 200u
#define US_PER_MS 1000u
#define HOTWIRE_TIMEOUT_GUARD_MS 2000u

/* All tuning #defines are in board_pins.h — see STEPPER_DEFAULT_STEP_DELAY_US,
 * SPAWN_*, AGITATOR_*, HOTWIRE_TRAVERSE_*, INDEXER_* etc. */

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

/* ── DRV8263 — flap (primary) ──────────────────────────────────────────────── */

static drv8263_t s_drv8263;
static bool s_drv8263_ready;

/* ── DRV8263 — flap (second instance) ─────────────────────────────────────── */

static drv8263_t s_drv8263_flap2;
static bool s_drv8263_flap2_ready = false;

/* ── DRV8263 — hot wire (secondary) ───────────────────────────────────────── */

static drv8263_t s_drv8263_hotwire;
static bool s_drv8263_hotwire_ready;

/* ── DRV8434S stepper chain & motion engine ────────────────────────────────── */

static drv8434s_chain_t s_stepper_chain;
static drv8434s_motion_t s_motion;
static struct repeating_timer s_step_timer;
static bool s_step_timer_active;
static uint32_t s_step_timer_period_us;
static bool s_stepper_ready;
static uint32_t s_spi_watchdog_last_ms = 0u; /* timestamp of last idle SPI health check */

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
static int32_t refUnit = HX711_REF_UNIT;   /* slope — set in board_pins.h */
static int32_t offset = HX711_ZERO_OFFSET; /* zero offset — set in board_pins.h */

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
static int32_t s_hotwire_pos_steps = 0;
static bool s_turntable_homed = false;
static int32_t s_indexer_pos_steps = 0; /* bag depth/eject rack (STEPPER_DEV_INDEXER) */

typedef enum
{
    ARM_HOME_IDLE = 0,
    ARM_HOME_SEEK,
    ARM_HOME_BACKOFF,
} arm_homing_phase_t;

typedef enum
{
    ARM_PRESS_RETRY_IDLE = 0,
    ARM_PRESS_RETRY_PRESSING,
    ARM_PRESS_RETRY_VERIFY,
    ARM_PRESS_RETRY_BACKOFF,
    ARM_PRESS_RETRY_REPRESSING,
} arm_press_retry_phase_t;

static arm_homing_phase_t s_arm_homing_phase = ARM_HOME_IDLE;
static bool s_arm_rehome_required = false;
static bool s_arm_homed = false;

typedef struct
{
    arm_press_retry_phase_t phase;
    uint8_t retries_started;
    uint16_t baseline_rpm;
    uint32_t phase_start_ms;
} arm_press_retry_t;

static arm_press_retry_t s_arm_press_retry = {
    .phase = ARM_PRESS_RETRY_IDLE,
};

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
static stepper_pending_t s_hotwire_pending;
static stepper_pending_t s_agitator_pending; /* MSG_AGITATE standalone cycle */

typedef enum
{
    AGIT_PHASE_IDLE            = 0,
    AGIT_PHASE_HOMING          = 1, /* driving toward mechanical home endstop */
    AGIT_PHASE_HOMING_BACKOFF  = 2, /* releasing from home endstop */
    AGIT_PHASE_FORWARD         = 3, /* forward stroke of a knead cycle */
    AGIT_PHASE_REVERSE         = 4, /* return stroke of a knead cycle */
} agitator_phase_t;

static agitator_phase_t s_agitator_phase      = AGIT_PHASE_IDLE;
static bool             s_agitator_cycle_done  = false; /* true when all cycles complete */
static bool             s_agitator_home_only   = false; /* true when an AGITATE_FLAG_DO_HOME request is in progress */
static uint8_t          s_agitator_cycles_done = 0u;    /* cycles completed so far */
static uint8_t          s_agitator_n_cycles    = 1u;    /* target number of fwd+rev cycles */
static int32_t          s_agitator_pos_steps   = 0;     /* position from home (signed) */

/* ── Vacuum pump state ─────────────────────────────────────────────────────── */

static bool s_vacuum_on = false;
static volatile uint32_t s_vacuum_pulse_count = 0; /* written by GPIO ISR */
static volatile uint32_t s_vacuum_last_pulse_time_us = 0u;
static volatile uint32_t s_vacuum_last_period_us = 0u;
static volatile uint64_t s_vacuum_period_sum_us = 0u;
static volatile uint32_t s_vacuum_period_samples = 0u;
static uint32_t s_vacuum_last_sample_ms = 0;
static uint32_t s_vacuum_last_status_ms = 0;
static vacuum_status_code_t s_vacuum_status = VACUUM_OFF;
static uint16_t s_vacuum_last_rpm = 0u;
static bool s_vacuum_rpm_valid = false;

/* ── Dosing callback ─────────────────────────────────────────────────────── */

/**
 * @brief Pico-side spawn dosing state machine states.
 *
 * States flow left-to-right.  Any state can transition to FAULT or ABORTED.
 *
 *  IDLE → [HOMING →] PRIME → DOSE_MAIN
 *                               ├─(Finish A)→ FINISH_A_CLOSE → CLOSE_CONFIRM
 *                               │                                    ├─ undershoot → FINISH_A_TOPOFF → [re-close] → DONE
 *                               │                                    └─ OK → DONE
 *                               └─(Finish B)→ FINISH_B_LOWFLOW → FAST_CLOSE → CLOSE_CONFIRM → DONE
 *
 *  DOSE_MAIN also transitions to FAST_CLOSE when dispensed_ug >= target_ug.
 */
typedef enum
{
    SPAWN_SM_IDLE = 0,
    SPAWN_SM_HOMING,          /* drive flaps to closed endpoint (re-zero virtual position) */
    SPAWN_SM_PRIME,           /* slowly open until flow detected                            */
    SPAWN_SM_DOSE_MAIN,       /* closed-loop proportional nudge dosing                      */
    SPAWN_SM_FINISH_A_CLOSE,  /* Finish A: issue early close; wait one tick for setup        */
    SPAWN_SM_CLOSE_CONFIRM,   /* wait for FLAP_SM_CLOSED or timeout                          */
    SPAWN_SM_FINISH_A_TOPOFF, /* Finish A: small open pulses to correct undershoot           */
    SPAWN_SM_FINISH_B_LOWFLOW,/* Finish B: minimal-pwm nudge near target                     */
    SPAWN_SM_FAST_CLOSE,      /* issue close at SPAWN_FAST_CLOSE_PWM                         */
    SPAWN_SM_DONE,
    SPAWN_SM_FAULT,
    SPAWN_SM_ABORTED,
    SPAWN_SM_AGIT_CLOSING,  /* agitation: waiting for flaps to fully close before running agitator */
    SPAWN_SM_AGITATING,     /* agitation: agitator forward+reverse cycle running; re-primes on done */
} spawn_sm_state_t;

/** What to do after CLOSE_CONFIRM completes. */
typedef enum
{
    CLOSE_NEXT_DONE = 0,        /* go straight to DONE */
    CLOSE_NEXT_CHECK_TOPOFF,    /* check for undershoot → maybe TOPOFF, else DONE */
} close_next_t;

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
    uint32_t window_start_ug; /* dispensed_ug at start of current flow window */
    uint32_t last_tick_ug;    /* dispensed_ug at the previous tick for nudge delta */
    uint32_t spawn_remaining_ug;
    absolute_time_t last_flow_check; /* timestamp of current window start */
    repeating_timer_t timer;
    bool nudging;                /* true while a position-nudge pulse is active */
    absolute_time_t nudge_until; /* absolute time when the current nudge should end */

    /* ── New state machine fields ────────────────────────────────────── */
    spawn_sm_state_t state;      /* current dosing SM state                   */
    spawn_sm_state_t prev_state; /* previous state (for transition logging)   */
    uint8_t finish_mode;         /* SPAWN_FINISH_MODE_A / SPAWN_FINISH_MODE_B */
    bool do_home;                /* home flaps to closed before priming        */

    /* EMA-filtered per-tick flow (µg/tick) */
    uint32_t ema_flow_ug;
    bool ema_seeded;

    /* Rate limiting */
    uint8_t consecutive_open_nudges; /* reset on close nudge or flow improvement */
    uint32_t prev_ema_flow_ug;       /* prior EMA value to detect improvement     */
    int8_t last_nudge_dir;           /* +1 open, -1 close, 0 unset                */
    absolute_time_t reverse_holdoff_until; /* block immediate opposite nudge     */

    /* Close confirmation */
    absolute_time_t close_start_time;
    close_next_t close_next; /* action after close confirm                      */

    /* Finish A top-off */
    uint8_t topoff_pulses;         /* count of top-off open pulses issued       */
    bool topoff_settling;          /* true during post-pulse settle wait        */
    absolute_time_t topoff_settle_until; /* end of settle window                */
} spawn_dose_ctx_t;

static spawn_dose_ctx_t s_spawn;

/* Spawn dosing constants are defined in board_pins.h with #ifndef guards.
 * Override them via -D flags or by editing board_pins.h before calibration. */

/* Derived spawn constants (computed from board_pins.h primaries) */
#define SPAWN_TICKS_PER_WINDOW (SPAWN_FLOW_WINDOW_MS / SPAWN_TIMER_PERIOD_MS)
#define SPAWN_TICK_MIN_UG (SPAWN_FLOW_MIN_UG / SPAWN_TICKS_PER_WINDOW)
#define SPAWN_TICK_MAX_UG (SPAWN_FLOW_MAX_UG / SPAWN_TICKS_PER_WINDOW)
/* Flap PWM values used during dosing (derived from board_pins.h flap speeds) */
#define SPAWN_OPEN_PWM (FLAP_OPEN_SPEED_PWM / 2)
#define SPAWN_REVERSE_PWM (FLAP_CLOSE_SPEED_PWM / 2)

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
    if (!s_drv8263_ready)
    {
        return false;
    }

    if (s_drv8263.monitoring_enabled)
    {
        drv8263_stop_current_monitoring(&s_drv8263);
    }
    s_drv8263.config.current_low_threshold = FLAP_OPEN_CURRENT_DROP_TH;
    s_drv8263.config.current_high_threshold = (uint16_t)FLAP_CLOSE_TORQUE_TH;
    s_drv8263.config.current_check_interval_ms = (uint32_t)FLAP_MONITOR_INTERVAL_MS;
    (void)drv8263_set_motor_control(&s_drv8263, DRV8263_MOTOR_REVERSE,
                                    (uint16_t)FLAP_CLOSE_SPEED_PWM);
    if (!drv8263_start_current_monitoring(&s_drv8263))
    {
        (void)drv8263_set_motor_control(&s_drv8263, DRV8263_MOTOR_STOP, 0u);
        return false;
    }

    if (s_drv8263_flap2_ready)
    {
        if (s_drv8263_flap2.monitoring_enabled)
        {
            drv8263_stop_current_monitoring(&s_drv8263_flap2);
        }
        s_drv8263_flap2.config.current_low_threshold = FLAP_OPEN_CURRENT_DROP_TH;
        s_drv8263_flap2.config.current_high_threshold = (uint16_t)FLAP_CLOSE_TORQUE_TH;
        s_drv8263_flap2.config.current_check_interval_ms = (uint32_t)FLAP_MONITOR_INTERVAL_MS;
        (void)drv8263_set_motor_control(&s_drv8263_flap2, DRV8263_MOTOR_REVERSE,
                                        (uint16_t)FLAP_CLOSE_SPEED_PWM);
        (void)drv8263_start_current_monitoring(&s_drv8263_flap2);
    }

    s_flap_state = FLAP_SM_CLOSING;
    s_flap_start_ms = to_ms_since_boot(get_absolute_time());
    s_flap1_stopped = false;
    s_flap2_stopped = (!s_drv8263_flap2_ready);
    return true;
}

/**
 * @brief Stop both flap motors immediately and halt any active current monitoring.
 *        Called during spawn dosing to immediately arrest motion before transitioning.
 */
static void spawn_stop_flaps(void)
{
    if (s_drv8263.monitoring_enabled)
    {
        drv8263_stop_current_monitoring(&s_drv8263);
    }
    (void)drv8263_set_motor_control(&s_drv8263, DRV8263_MOTOR_STOP, 0u);

    if (s_drv8263_flap2_ready)
    {
        if (s_drv8263_flap2.monitoring_enabled)
        {
            drv8263_stop_current_monitoring(&s_drv8263_flap2);
        }
        (void)drv8263_set_motor_control(&s_drv8263_flap2, DRV8263_MOTOR_STOP, 0u);
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
 * @brief Issue a fast close for end-of-dose by engaging torque-monitored close
 *        at SPAWN_FAST_CLOSE_PWM (higher PWM than normal close).
 *        Records close_start_time in s_spawn for the CLOSE_CONFIRM state.
 */
static void spawn_fast_close(void)
{
    spawn_stop_flaps();

    if (!s_drv8263_ready)
    {
        return;
    }

    if (s_drv8263.monitoring_enabled)
    {
        drv8263_stop_current_monitoring(&s_drv8263);
    }
    s_drv8263.config.current_low_threshold  = FLAP_OPEN_CURRENT_DROP_TH;
    s_drv8263.config.current_high_threshold = (uint16_t)FLAP_CLOSE_TORQUE_TH;
    s_drv8263.config.current_check_interval_ms = (uint32_t)FLAP_MONITOR_INTERVAL_MS;
    (void)drv8263_set_motor_control(&s_drv8263, DRV8263_MOTOR_REVERSE,
                                    (uint16_t)SPAWN_FAST_CLOSE_PWM);
    if (!drv8263_start_current_monitoring(&s_drv8263))
    {
        (void)drv8263_set_motor_control(&s_drv8263, DRV8263_MOTOR_STOP, 0u);
    }

    if (s_drv8263_flap2_ready)
    {
        if (s_drv8263_flap2.monitoring_enabled)
        {
            drv8263_stop_current_monitoring(&s_drv8263_flap2);
        }
        s_drv8263_flap2.config.current_low_threshold  = FLAP_OPEN_CURRENT_DROP_TH;
        s_drv8263_flap2.config.current_high_threshold = (uint16_t)FLAP_CLOSE_TORQUE_TH;
        s_drv8263_flap2.config.current_check_interval_ms = (uint32_t)FLAP_MONITOR_INTERVAL_MS;
        (void)drv8263_set_motor_control(&s_drv8263_flap2, DRV8263_MOTOR_REVERSE,
                                        (uint16_t)SPAWN_FAST_CLOSE_PWM);
        (void)drv8263_start_current_monitoring(&s_drv8263_flap2);
    }

    s_flap_state  = FLAP_SM_CLOSING;
    s_flap_start_ms = to_ms_since_boot(get_absolute_time());
    s_flap1_stopped = false;
    s_flap2_stopped = (!s_drv8263_flap2_ready);

    s_spawn.close_start_time = get_absolute_time();
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
    drv8263_motor_state_t st = forward ? DRV8263_MOTOR_FORWARD : DRV8263_MOTOR_REVERSE;
    (void)drv8263_set_motor_control(&s_drv8263, st, pwm);
    if (s_drv8263_flap2_ready)
    {
        (void)drv8263_set_motor_control(&s_drv8263_flap2, st, pwm);
    }
}

static bool spawn_direction_reversal_blocked(int8_t desired_dir, absolute_time_t now)
{
    bool opposite = ((desired_dir > 0 && s_spawn.last_nudge_dir < 0) ||
                     (desired_dir < 0 && s_spawn.last_nudge_dir > 0));
    if (!opposite)
    {
        return false;
    }

    return absolute_time_diff_us(s_spawn.reverse_holdoff_until, now) < 0;
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

/**
 * @brief Transition the spawn dosing SM to a new state and log the transition.
 */
static void spawn_set_state(spawn_sm_state_t next)
{
    if (s_spawn.state != next)
    {
        s_spawn.prev_state = s_spawn.state;
        s_spawn.state = next;
        printf("Spawn SM: %u → %u\n", (unsigned)s_spawn.prev_state, (unsigned)next);
    }
}

static void spawn_stop_timer(void)
{
    (void)cancel_repeating_timer(&s_spawn.timer);
    s_spawn.timer.delay_us = 0;
}

/* Forward declaration — ensure_step_timer_running is defined after the
 * DRV8434S callback section but called from dispense_spawn_callback. */
static bool ensure_step_timer_running(uint32_t step_delay_us);
static bool start_stepper_job(uint8_t dev_idx, int32_t relative_steps,
                              uint16_t torque_limit,
                              uint16_t torque_blank_steps,
                              uint32_t step_delay_us);
static bool agitator_start_homing(void); /* sensorless home then start cycles */
static bool agitator_start_cycle(void);  /* N×(forward+reverse) knead; shared by spawn SM and MSG_AGITATE */
static bool start_arm_internal_absolute_move(int32_t target_steps,
                                             uint16_t torque_limit,
                                             uint32_t timeout_ms);
static bool start_arm_internal_relative_move(int32_t relative_steps,
                                             uint16_t torque_limit,
                                             uint32_t timeout_ms);
static bool start_arm_home_backoff(void);
static void arm_press_retry_tick(void);

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
        /* Stop motor; hold current flap position until next nudge decision.
         * Must disable current monitoring to prevent false undercurrent flags
         * when the motor is intentionally stopped (zero current is normal). */
        if (s_drv8263.monitoring_enabled)
        {
            drv8263_stop_current_monitoring(&s_drv8263);
        }
        (void)drv8263_set_motor_control(&s_drv8263, DRV8263_MOTOR_STOP, 0u);
        
        if (s_drv8263_flap2_ready)
        {
            if (s_drv8263_flap2.monitoring_enabled)
            {
                drv8263_stop_current_monitoring(&s_drv8263_flap2);
            }
            (void)drv8263_set_motor_control(&s_drv8263_flap2, DRV8263_MOTOR_STOP, 0u);
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
        /* Clamp via int64_t to avoid undefined behaviour if mass_g > 2147 g
         * (e.g. untared scale) which would overflow int32_t.  Also clamp the
         * double before the int64 cast in case mass_g is astronomically large
         * (e.g. sensor fault) to stay within valid int64_t range. */
        static const double K_MAX_UG = (double)INT32_MAX;
        static const double K_MIN_UG = (double)INT32_MIN;
        double mass_ug_d = mass_g * 1000000.0;
        if (mass_ug_d > K_MAX_UG) mass_ug_d = K_MAX_UG;
        if (mass_ug_d < K_MIN_UG) mass_ug_d = K_MIN_UG;
        mass.ug = (int32_t)(int64_t)mass_ug_d;
        mass.unit = s_scale.unit;
    }

    /* Use non-negative mass only: negative means below tare (nothing dispensed yet) */
    uint32_t current_ug = (mass.ug >= 0) ? (uint32_t)mass.ug : 0u;
    double m = 0;
    mass_get_value(&mass, &m);

    s_spawn.dispensed_ug = (current_ug > s_spawn.start_mass_ug)
                               ? (current_ug - s_spawn.start_mass_ug)
                               : 0u;

    /* ── Per-tick delta — spike-clamped then EMA-filtered ──────────────────── */
    uint32_t tick_delta_ug = (s_spawn.dispensed_ug > s_spawn.last_tick_ug)
                                 ? (s_spawn.dispensed_ug - s_spawn.last_tick_ug)
                                 : 0u;
    s_spawn.last_tick_ug = s_spawn.dispensed_ug;

    /* Spike clamp: ignore implausibly large jumps (sensor glitch / reset) */
    if (tick_delta_ug > (uint32_t)SPAWN_FLOW_SPIKE_CLAMP_UG)
    {
        printf("Spawn: spike clamped tick_delta=%lu → 0\n", (unsigned long)tick_delta_ug);
        tick_delta_ug = 0u;
    }

    /* EMA update: ema = alpha*delta + (1-alpha)*ema, using integer maths */
    if (!s_spawn.ema_seeded)
    {
        s_spawn.ema_flow_ug = tick_delta_ug;
        s_spawn.ema_seeded  = true;
    }
    else
    {
        /* ema = (alpha_x1000 * tick_delta + (1000-alpha_x1000) * ema) / 1000 */
        s_spawn.ema_flow_ug =
            ((uint32_t)SPAWN_EMA_ALPHA_X1000 * tick_delta_ug +
             (1000u - (uint32_t)SPAWN_EMA_ALPHA_X1000) * s_spawn.ema_flow_ug) / 1000u;
    }

    uint32_t remaining_ug = (s_spawn.dispensed_ug < s_spawn.target_ug)
                                ? (s_spawn.target_ug - s_spawn.dispensed_ug)
                                : 0u;

    printf("Spawn[%u]: mass=%.3fg disp=%luug remain=%luug ema_flow=%luug/tick\n",
           (unsigned)s_spawn.state, m,
           (unsigned long)s_spawn.dispensed_ug,
           (unsigned long)remaining_ug,
           (unsigned long)s_spawn.ema_flow_ug);

    /* ══════════════════════════════════════════════════════════════════════════ *
     *  State machine                                                             *
     * ══════════════════════════════════════════════════════════════════════════ */

    switch (s_spawn.state)
    {

    /* ── HOMING: drive to fully-closed endpoint, wait for confirmation ──────── */
    case SPAWN_SM_HOMING:
    {
        uint32_t elapsed_ms = (uint32_t)(absolute_time_diff_us(s_spawn.close_start_time, now) / 1000);
        if (s_flap_state == FLAP_SM_CLOSED)
        {
            printf("Spawn HOMING: closed endpoint confirmed in %lums\n", (unsigned long)elapsed_ms);
            /* Transition to PRIME — re-arm undercurrent monitoring for startup open */
            s_spawn.startup_opening = true;
            s_spawn.last_flow_check = get_absolute_time();
            s_spawn.window_start_ug = s_spawn.dispensed_ug;
            s_spawn.last_tick_ug    = s_spawn.dispensed_ug;
            s_spawn.ema_seeded      = false;
            s_spawn.ema_flow_ug     = 0u;

            s_drv8263.config.current_low_threshold     = (uint16_t)FLAP_OPEN_CURRENT_DROP_TH;
            s_drv8263.config.current_high_threshold    = 4095u;
            s_drv8263.config.current_check_interval_ms = (uint32_t)FLAP_MONITOR_INTERVAL_MS;
            spawn_set_flap_pwm(true, (uint16_t)SPAWN_OPEN_PWM);
            (void)drv8263_start_current_monitoring(&s_drv8263);
            if (s_drv8263_flap2_ready)
            {
                s_drv8263_flap2.config.current_low_threshold     = (uint16_t)FLAP_OPEN_CURRENT_DROP_TH;
                s_drv8263_flap2.config.current_high_threshold    = 4095u;
                s_drv8263_flap2.config.current_check_interval_ms = (uint32_t)FLAP_MONITOR_INTERVAL_MS;
                (void)drv8263_start_current_monitoring(&s_drv8263_flap2);
            }
            spawn_set_state(SPAWN_SM_PRIME);
            send_spawn_status(SPAWN_STATUS_RUNNING);
        }
        else if (elapsed_ms >= (uint32_t)SPAWN_HOME_TIMEOUT_MS)
        {
            printf("Spawn HOMING: timeout after %lums — proceeding to PRIME\n",
                   (unsigned long)elapsed_ms);
            /* Fail-safe: assume we are closed enough and proceed */
            s_spawn.startup_opening = true;
            s_spawn.last_flow_check = get_absolute_time();
            s_spawn.window_start_ug = s_spawn.dispensed_ug;
            s_spawn.last_tick_ug    = s_spawn.dispensed_ug;
            s_spawn.ema_seeded      = false;
            s_spawn.ema_flow_ug     = 0u;

            s_drv8263.config.current_low_threshold     = (uint16_t)FLAP_OPEN_CURRENT_DROP_TH;
            s_drv8263.config.current_high_threshold    = 4095u;
            s_drv8263.config.current_check_interval_ms = (uint32_t)FLAP_MONITOR_INTERVAL_MS;
            spawn_set_flap_pwm(true, (uint16_t)SPAWN_OPEN_PWM);
            (void)drv8263_start_current_monitoring(&s_drv8263);
            if (s_drv8263_flap2_ready)
            {
                s_drv8263_flap2.config.current_low_threshold     = (uint16_t)FLAP_OPEN_CURRENT_DROP_TH;
                s_drv8263_flap2.config.current_high_threshold    = 4095u;
                s_drv8263_flap2.config.current_check_interval_ms = (uint32_t)FLAP_MONITOR_INTERVAL_MS;
                (void)drv8263_start_current_monitoring(&s_drv8263_flap2);
            }
            spawn_set_state(SPAWN_SM_PRIME);
            send_spawn_status(SPAWN_STATUS_RUNNING);
        }
        /* else: still homing — motor is running, just wait */
        return true;
    }

    /* ── PRIME: drive open slowly until SPAWN_STARTUP_FLOW_DETECT_UG detected ─ */
    case SPAWN_SM_PRIME:
    {
        /* Undercurrent = reached end-of-travel before any flow: flow failure */
        bool startup_undercurrent = s_spawn.startup_opening &&
                                    s_drv8263.current_status == DRV8263_CURRENT_UNDERCURRENT &&
                                    tick_delta_ug < SPAWN_STARTUP_FLOW_DETECT_UG;
        if (startup_undercurrent)
        {
            printf("Spawn PRIME: end-of-travel, no flow — FLOW_FAILURE\n");
            spawn_close_flaps();
            s_spawn.active = false;
            spawn_set_state(SPAWN_SM_FAULT);
            send_spawn_status(SPAWN_STATUS_FLOW_FAILURE);
            return false;
        }

        if (tick_delta_ug >= SPAWN_STARTUP_FLOW_DETECT_UG)
        {
            /* Flow confirmed — stop undercurrent monitoring, park motor */
            drv8263_stop_current_monitoring(&s_drv8263);
            if (s_drv8263_flap2_ready && s_drv8263_flap2.monitoring_enabled)
            {
                drv8263_stop_current_monitoring(&s_drv8263_flap2);
            }
            s_spawn.startup_opening = false;
            s_spawn.nudging         = false;
            (void)drv8263_set_motor_control(&s_drv8263, DRV8263_MOTOR_STOP, 0u);
            if (s_drv8263_flap2_ready)
            {
                (void)drv8263_set_motor_control(&s_drv8263_flap2, DRV8263_MOTOR_STOP, 0u);
            }
            /* Reset flow window so budget begins from first confirmed flow */
            s_spawn.last_flow_check = get_absolute_time();
            s_spawn.window_start_ug = s_spawn.dispensed_ug;
            s_spawn.ema_seeded      = false;
            s_spawn.ema_flow_ug     = 0u;
            s_spawn.consecutive_open_nudges = 0u;
            s_spawn.prev_ema_flow_ug        = 0u;
            printf("Spawn PRIME→DOSE_MAIN: flow detected tick_delta=%luug\n",
                   (unsigned long)tick_delta_ug);
            spawn_set_state(SPAWN_SM_DOSE_MAIN);
            send_spawn_status(SPAWN_STATUS_RUNNING);
            /* Fall through to DOSE_MAIN on this same tick */
        }
        else
        {
            /* No weight change yet — keep driving open */
            spawn_set_flap_pwm(true, (uint16_t)SPAWN_OPEN_PWM);
            return true;
        }
        /* FALL THROUGH to DOSE_MAIN */
    }
    /* fall through */

    /* ── DOSE_MAIN: closed-loop proportional nudge with finish detection ─────── */
    case SPAWN_SM_DOSE_MAIN:
    {
        /* ── Flow window: agitation / no-flow retry ─────────────────────────── */
        now = get_absolute_time();
        if (absolute_time_diff_us(s_spawn.last_flow_check, now) >=
            (int64_t)((uint32_t)SPAWN_FLOW_WINDOW_MS * 1000u))
        {
            uint32_t window_delta = (s_spawn.dispensed_ug > s_spawn.window_start_ug)
                                        ? (s_spawn.dispensed_ug - s_spawn.window_start_ug)
                                        : 0u;

            if (window_delta < (uint32_t)SPAWN_FLOW_NOFLOW_UG)
            {
                s_spawn.retry++;
                printf("Spawn DOSE_MAIN: no-flow (delta=%lu ug) — closing flaps for agitation (retry %u/%u)\n",
                       (unsigned long)window_delta, (unsigned)s_spawn.retry,
                       (unsigned)s_spawn.max_retries);
                send_spawn_status(SPAWN_STATUS_AGITATING);
                /* Fully close flaps before running the agitator, then re-prime */
                spawn_fast_close();
                s_spawn.close_start_time = get_absolute_time();
                s_spawn.window_start_ug  = s_spawn.dispensed_ug;
                spawn_set_state(SPAWN_SM_AGIT_CLOSING);
                return true;
            }

            /* Window completed normally — reset window tracking */
            s_spawn.window_start_ug = s_spawn.dispensed_ug;
            s_spawn.last_flow_check = now;

        }

        /* ── Nudge undercurrent: flap reached fully-open endpoint ───────────── */
        bool nudge_undercurrent = s_spawn.nudging &&
                                  s_drv8263.current_status == DRV8263_CURRENT_UNDERCURRENT;
        if (nudge_undercurrent)
        {
            printf("Spawn DOSE_MAIN: flap fully open (undercurrent during nudge) — FLOW_FAILURE\n");
            spawn_close_flaps();
            s_spawn.active = false;
            spawn_set_state(SPAWN_SM_FAULT);
            send_spawn_status(SPAWN_STATUS_FLOW_FAILURE);
            return false;
        }

        /* ── Target reached — fast close ────────────────────────────────────── */
        if (s_spawn.dispensed_ug >= s_spawn.target_ug)
        {
            printf("Spawn DOSE_MAIN: target reached (%luug) — FAST_CLOSE\n",
                   (unsigned long)s_spawn.dispensed_ug);
            s_spawn.close_next = CLOSE_NEXT_DONE;
            spawn_fast_close();
            printf("Spawn FAST_CLOSE: fast close issued (PWM=%u)\n",
                   (unsigned)SPAWN_FAST_CLOSE_PWM);
            spawn_set_state(SPAWN_SM_CLOSE_CONFIRM);
            return true;
        }

        /* ── Finish mode transition check ───────────────────────────────────── */
        if (s_spawn.finish_mode == (uint8_t)SPAWN_FINISH_MODE_A)
        {
            /* Finish A: predict in-flight overshoot and close early.
             * predicted_in_flight_ug = ema_flow_ug * latency_ticks
             * latency_ticks = SPAWN_CLOSE_LATENCY_MS / SPAWN_TIMER_PERIOD_MS   */
            uint32_t latency_ticks = (uint32_t)SPAWN_CLOSE_LATENCY_MS /
                                     (uint32_t)SPAWN_TIMER_PERIOD_MS;
            uint32_t predicted_ug = s_spawn.ema_flow_ug * latency_ticks;
            uint32_t trigger_ug   = predicted_ug + (uint32_t)SPAWN_CLOSE_EARLY_MARGIN_UG;

            if (remaining_ug <= trigger_ug && s_spawn.ema_seeded)
            {
                printf("Spawn DOSE_MAIN→FINISH_A_CLOSE: remain=%luug predicted=%luug trigger=%luug\n",
                       (unsigned long)remaining_ug, (unsigned long)predicted_ug,
                       (unsigned long)trigger_ug);
                s_spawn.close_next = CLOSE_NEXT_CHECK_TOPOFF;
                spawn_fast_close();
                printf("Spawn FINISH_A_CLOSE: fast close issued (PWM=%u)\n",
                       (unsigned)SPAWN_FAST_CLOSE_PWM);
                spawn_set_state(SPAWN_SM_CLOSE_CONFIRM);
                return true;
            }
        }
        else /* SPAWN_FINISH_MODE_B */
        {
            /* Finish B: transition to low-flow taper when near target */
            if (remaining_ug <= (uint32_t)SPAWN_LOWFLOW_THRESHOLD_UG)
            {
                printf("Spawn DOSE_MAIN→FINISH_B_LOWFLOW: remain=%luug\n",
                       (unsigned long)remaining_ug);
                spawn_set_state(SPAWN_SM_FINISH_B_LOWFLOW);
                return true; /* low-flow runs on next tick */
            }
        }

        /* ── Proportional per-tick target rate ──────────────────────────────── *
         *                                                                        *
         *   remaining > target/UPPER → desired = SPAWN_TICK_MAX_UG              *
         *   remaining < target/LOWER → desired = SPAWN_TICK_MIN_UG              *
         *   in between               → linear interpolation                      */
        {
            uint32_t upper_ug = s_spawn.target_ug / (uint32_t)SPAWN_PROP_UPPER;
            uint32_t lower_ug = s_spawn.target_ug / (uint32_t)SPAWN_PROP_LOWER;
            uint32_t desired_tick_ug;

            if (remaining_ug >= upper_ug)
            {
                desired_tick_ug = SPAWN_TICK_MAX_UG;
            }
            else if (remaining_ug > lower_ug)
            {
                uint32_t range   = upper_ug - lower_ug;
                uint32_t frac    = remaining_ug - lower_ug;
                desired_tick_ug  = SPAWN_TICK_MIN_UG +
                                   ((uint32_t)(SPAWN_TICK_MAX_UG - SPAWN_TICK_MIN_UG) * frac) / range;
            }
            else
            {
                desired_tick_ug = SPAWN_TICK_MIN_UG;
            }

            /* ── Incremental position nudge (rate-limited) ─────────────────── */
            if (!s_spawn.nudging)
            {
                int32_t error_ug = (int32_t)desired_tick_ug - (int32_t)s_spawn.ema_flow_ug;

                if (error_ug > (int32_t)SPAWN_TICK_DEADBAND)
                {
                    /* Flow below target — open a little, with rate limit */
                    s_spawn.consecutive_open_nudges++;
                    if (s_spawn.consecutive_open_nudges > (uint8_t)SPAWN_MAX_OPEN_NUDGES)
                    {
                        printf("Spawn DOSE_MAIN: max open nudges (%u) hit, holding position\n",
                               (unsigned)SPAWN_MAX_OPEN_NUDGES);
                        s_spawn.consecutive_open_nudges = 0u; /* reset after hold */
                    }
                    else if (spawn_direction_reversal_blocked(+1, now))
                    {
                        printf("Spawn DOSE_MAIN: holdoff blocks OPEN reversal (ema=%luug)\n",
                               (unsigned long)s_spawn.ema_flow_ug);
                    }
                    else
                    {
                        spawn_set_flap_pwm(true, (uint16_t)SPAWN_OPEN_PWM);
                        s_spawn.nudge_until = delayed_by_ms(get_absolute_time(),
                                                            (uint32_t)SPAWN_NUDGE_OPEN_MS);
                        s_spawn.nudging = true;
                        s_spawn.last_nudge_dir = +1;
                        s_spawn.reverse_holdoff_until = delayed_by_ms(now,
                                                                       (uint32_t)SPAWN_DIRECTION_REVERSAL_HOLDOFF_MS);
                        printf("Spawn DOSE_MAIN: open nudge %ums (error=%ldug, ema=%luug, consec=%u)\n",
                               (unsigned)SPAWN_NUDGE_OPEN_MS,
                               (long)error_ug,
                               (unsigned long)s_spawn.ema_flow_ug,
                               (unsigned)s_spawn.consecutive_open_nudges);
                    }
                }
                else if (error_ug < -(int32_t)SPAWN_TICK_DEADBAND)
                {
                    /* Flow above target — close a little; reset open nudge counter */
                    s_spawn.consecutive_open_nudges = 0u;
                    if (spawn_direction_reversal_blocked(-1, now))
                    {
                        printf("Spawn DOSE_MAIN: holdoff blocks CLOSE reversal (ema=%luug)\n",
                               (unsigned long)s_spawn.ema_flow_ug);
                    }
                    else
                    {
                        spawn_set_flap_pwm(false, (uint16_t)SPAWN_REVERSE_PWM);
                        s_spawn.nudge_until = delayed_by_ms(get_absolute_time(),
                                                            (uint32_t)SPAWN_NUDGE_CLOSE_MS);
                        s_spawn.nudging = true;
                        s_spawn.last_nudge_dir = -1;
                        s_spawn.reverse_holdoff_until = delayed_by_ms(now,
                                                                       (uint32_t)SPAWN_DIRECTION_REVERSAL_HOLDOFF_MS);
                        printf("Spawn DOSE_MAIN: close nudge %ums (error=%ldug, ema=%luug)\n",
                               (unsigned)SPAWN_NUDGE_CLOSE_MS,
                               (long)error_ug,
                               (unsigned long)s_spawn.ema_flow_ug);
                    }
                }
                /* else: within deadband — hold position */

                /* Reset open nudge counter if EMA flow has improved */
                if (s_spawn.ema_flow_ug > s_spawn.prev_ema_flow_ug)
                {
                    s_spawn.consecutive_open_nudges = 0u;
                }
                s_spawn.prev_ema_flow_ug = s_spawn.ema_flow_ug;
            }
        }
        return true;
    }

    /* ── FINISH_A_CLOSE: issue fast close, then wait for confirm ─────────────── */
    case SPAWN_SM_FINISH_A_CLOSE:
    {
        s_spawn.close_next = CLOSE_NEXT_CHECK_TOPOFF;
        spawn_fast_close();
        printf("Spawn FINISH_A_CLOSE: fast close issued (PWM=%u)\n",
               (unsigned)SPAWN_FAST_CLOSE_PWM);
        spawn_set_state(SPAWN_SM_CLOSE_CONFIRM);
        return true;
    }

    /* ── FINISH_B_LOWFLOW: minimal nudges near target, then fast-close ───────── */
    case SPAWN_SM_FINISH_B_LOWFLOW:
    {
        /* Close fully once remaining drops to SPAWN_CLOSE_THRESHOLD_UG */
        if (remaining_ug <= (uint32_t)SPAWN_CLOSE_THRESHOLD_UG || s_spawn.dispensed_ug >= s_spawn.target_ug)
        {
            printf("Spawn FINISH_B_LOWFLOW: close threshold reached (remain=%luug) — FAST_CLOSE\n",
                   (unsigned long)remaining_ug);
            s_spawn.close_next = CLOSE_NEXT_DONE;
            spawn_fast_close();
            printf("Spawn FAST_CLOSE: fast close issued (PWM=%u)\n",
                   (unsigned)SPAWN_FAST_CLOSE_PWM);
            spawn_set_state(SPAWN_SM_CLOSE_CONFIRM);
            return true;
        }

        /* Low-flow nudge: open for SPAWN_LOWFLOW_NUDGE_MS if below desired */
        if (!s_spawn.nudging)
        {
            /* In low-flow mode desired rate = SPAWN_TICK_MIN_UG */
            int32_t error_ug = (int32_t)SPAWN_TICK_MIN_UG - (int32_t)s_spawn.ema_flow_ug;
            if (error_ug > (int32_t)SPAWN_TICK_DEADBAND)
            {
                if (spawn_direction_reversal_blocked(+1, now))
                {
                    printf("Spawn FINISH_B_LOWFLOW: holdoff blocks OPEN reversal (ema=%luug)\n",
                           (unsigned long)s_spawn.ema_flow_ug);
                }
                else
                {
                    spawn_set_flap_pwm(true, (uint16_t)SPAWN_OPEN_PWM);
                    s_spawn.nudge_until = delayed_by_ms(get_absolute_time(),
                                                        (uint32_t)SPAWN_LOWFLOW_NUDGE_MS);
                    s_spawn.nudging = true;
                    s_spawn.last_nudge_dir = +1;
                    s_spawn.reverse_holdoff_until = delayed_by_ms(now,
                                                                   (uint32_t)SPAWN_DIRECTION_REVERSAL_HOLDOFF_MS);
                    printf("Spawn FINISH_B_LOWFLOW: open nudge %ums (ema=%luug)\n",
                           (unsigned)SPAWN_LOWFLOW_NUDGE_MS,
                           (unsigned long)s_spawn.ema_flow_ug);
                }
            }
            else if (error_ug < -(int32_t)SPAWN_TICK_DEADBAND)
            {
                if (spawn_direction_reversal_blocked(-1, now))
                {
                    printf("Spawn FINISH_B_LOWFLOW: holdoff blocks CLOSE reversal (ema=%luug)\n",
                           (unsigned long)s_spawn.ema_flow_ug);
                }
                else
                {
                    spawn_set_flap_pwm(false, (uint16_t)SPAWN_REVERSE_PWM);
                    s_spawn.nudge_until = delayed_by_ms(get_absolute_time(),
                                                        (uint32_t)SPAWN_NUDGE_CLOSE_MS);
                    s_spawn.nudging = true;
                    s_spawn.last_nudge_dir = -1;
                    s_spawn.reverse_holdoff_until = delayed_by_ms(now,
                                                                   (uint32_t)SPAWN_DIRECTION_REVERSAL_HOLDOFF_MS);
                    printf("Spawn FINISH_B_LOWFLOW: close nudge (ema=%luug)\n",
                           (unsigned long)s_spawn.ema_flow_ug);
                }
            }
        }
        return true;
    }

    /* ── FAST_CLOSE: issue fast close at high PWM ─────────────────────────────── */
    case SPAWN_SM_FAST_CLOSE:
    {
        s_spawn.close_next = CLOSE_NEXT_DONE;
        spawn_fast_close();
        printf("Spawn FAST_CLOSE: fast close issued (PWM=%u)\n",
               (unsigned)SPAWN_FAST_CLOSE_PWM);
        spawn_set_state(SPAWN_SM_CLOSE_CONFIRM);
        return true;
    }

    /* ── CLOSE_CONFIRM: wait for FLAP_SM_CLOSED or timeout ──────────────────── */
    case SPAWN_SM_CLOSE_CONFIRM:
    {
        uint32_t elapsed_ms = (uint32_t)(absolute_time_diff_us(s_spawn.close_start_time, now) / 1000);
        bool close_done = (s_flap_state == FLAP_SM_CLOSED || s_flap_state == FLAP_SM_FAULT);
        bool timed_out  = (elapsed_ms >= (uint32_t)SPAWN_CLOSE_CONFIRM_TIMEOUT_MS);

        if (!close_done && !timed_out)
        {
            return true; /* still waiting */
        }

        if (timed_out && !close_done)
        {
            printf("Spawn CLOSE_CONFIRM: timeout after %lums (flap_state=%u)\n",
                   (unsigned long)elapsed_ms, (unsigned)s_flap_state);
        }
        else
        {
            printf("Spawn CLOSE_CONFIRM: closed in %lums (flap_state=%u)\n",
                   (unsigned long)elapsed_ms, (unsigned)s_flap_state);
        }

        if (s_spawn.close_next == CLOSE_NEXT_CHECK_TOPOFF)
        {
            /* Finish A: check if we undershot by more than SPAWN_TOPOFF_TOLERANCE_UG */
            if (s_spawn.dispensed_ug < s_spawn.target_ug &&
                (s_spawn.target_ug - s_spawn.dispensed_ug) > (uint32_t)SPAWN_TOPOFF_TOLERANCE_UG)
            {
                printf("Spawn CLOSE_CONFIRM: undershoot %luug > tolerance %u — TOPOFF\n",
                       (unsigned long)(s_spawn.target_ug - s_spawn.dispensed_ug),
                       (unsigned)SPAWN_TOPOFF_TOLERANCE_UG);
                s_spawn.topoff_pulses   = 0u;
                s_spawn.topoff_settling = false;
                spawn_set_state(SPAWN_SM_FINISH_A_TOPOFF);
                return true;
            }
        }

        /* Done */
        s_spawn.active = false;
        printf("Spawn: DONE — dispensed=%luug target=%luug\n",
               (unsigned long)s_spawn.dispensed_ug, (unsigned long)s_spawn.target_ug);
        spawn_set_state(SPAWN_SM_DONE);
        send_spawn_status(SPAWN_STATUS_DONE);
        return false;
    }

    /* ── AGIT_CLOSING: flaps closing before agitation — wait for FLAP_SM_CLOSED ─ */
    case SPAWN_SM_AGIT_CLOSING:
    {
        uint32_t elapsed_ms = (uint32_t)(absolute_time_diff_us(s_spawn.close_start_time, now) / 1000u);
        bool close_done = (s_flap_state == FLAP_SM_CLOSED || s_flap_state == FLAP_SM_FAULT);
        bool timed_out  = (elapsed_ms >= (uint32_t)SPAWN_CLOSE_CONFIRM_TIMEOUT_MS);

        if (!close_done && !timed_out)
        {
            return true; /* still closing */
        }

        printf("Spawn AGIT_CLOSING: flaps %s in %lums — starting agitator\n",
               close_done ? "closed" : "timed-out", (unsigned long)elapsed_ms);

        /* Start the N-cycle knead (forward + reverse × AGITATOR_N_CYCLES) */
        s_agitator_n_cycles    = (uint8_t)AGITATOR_N_CYCLES;
        s_agitator_cycles_done = 0u;
        s_agitator_cycle_done  = false;
        if (!agitator_start_cycle())
        {
            /* Agitator not wired or failed — skip to retry/re-prime directly */
            printf("Spawn AGIT_CLOSING: agitator not available — re-priming\n");
            s_agitator_cycle_done = true; /* treat as done to fall through */
        }
        spawn_set_state(SPAWN_SM_AGITATING);
        return true;
    }

    /* ── AGITATING: agitator cycle running — wait for completion then re-prime ── */
    case SPAWN_SM_AGITATING:
    {
        if (!s_agitator_cycle_done)
        {
            return true; /* agitator still running */
        }

        printf("Spawn AGITATING: cycle done — retry=%u/%u\n",
               (unsigned)s_spawn.retry, (unsigned)s_spawn.max_retries);

        if (s_spawn.retry >= s_spawn.max_retries)
        {
            s_spawn.active = false;
            printf("Spawn AGITATING: max retries reached — BAG_EMPTY\n");
            spawn_set_state(SPAWN_SM_FAULT);
            send_spawn_status(SPAWN_STATUS_BAG_EMPTY);
            return false;
        }

        /* Re-engage prime sequence: open flaps and look for fresh flow */
        s_spawn.startup_opening = true;
        s_spawn.last_flow_check = get_absolute_time();
        s_spawn.window_start_ug = s_spawn.dispensed_ug;
        s_spawn.last_tick_ug    = s_spawn.dispensed_ug;
        s_spawn.ema_seeded      = false;
        s_spawn.ema_flow_ug     = 0u;
        s_spawn.consecutive_open_nudges = 0u;

        s_drv8263.config.current_low_threshold      = (uint16_t)FLAP_OPEN_CURRENT_DROP_TH;
        s_drv8263.config.current_high_threshold     = 4095u;
        s_drv8263.config.current_check_interval_ms  = (uint32_t)FLAP_MONITOR_INTERVAL_MS;
        spawn_set_flap_pwm(true, (uint16_t)SPAWN_OPEN_PWM);
        (void)drv8263_start_current_monitoring(&s_drv8263);
        if (s_drv8263_flap2_ready)
        {
            s_drv8263_flap2.config.current_low_threshold     = (uint16_t)FLAP_OPEN_CURRENT_DROP_TH;
            s_drv8263_flap2.config.current_high_threshold    = 4095u;
            s_drv8263_flap2.config.current_check_interval_ms = (uint32_t)FLAP_MONITOR_INTERVAL_MS;
            (void)drv8263_start_current_monitoring(&s_drv8263_flap2);
        }
        spawn_set_state(SPAWN_SM_PRIME);
        send_spawn_status(SPAWN_STATUS_RUNNING);
        return true;
    }

    /* ── FINISH_A_TOPOFF: small open pulses to correct undershoot ────────────── */
    case SPAWN_SM_FINISH_A_TOPOFF:
    {
        /* Check if tolerance met or max pulses reached */
        bool within_tol = (s_spawn.dispensed_ug >= s_spawn.target_ug) ||
                          (s_spawn.target_ug > s_spawn.dispensed_ug &&
                           (s_spawn.target_ug - s_spawn.dispensed_ug) <=
                               (uint32_t)SPAWN_TOPOFF_TOLERANCE_UG);

        if (within_tol || s_spawn.topoff_pulses >= (uint8_t)SPAWN_TOPOFF_MAX_PULSES)
        {
            printf("Spawn TOPOFF: done (%s) pulses=%u dispensed=%luug target=%luug\n",
                   within_tol ? "within tol" : "max pulses",
                   (unsigned)s_spawn.topoff_pulses,
                   (unsigned long)s_spawn.dispensed_ug,
                   (unsigned long)s_spawn.target_ug);
            s_spawn.active = false;
            spawn_set_state(SPAWN_SM_DONE);
            send_spawn_status(SPAWN_STATUS_DONE);
            return false;
        }

        /* During settle wait — keep flaps stopped */
        if (s_spawn.topoff_settling)
        {
            if (absolute_time_diff_us(s_spawn.topoff_settle_until, now) >= 0)
            {
                s_spawn.topoff_settling = false;
                printf("Spawn TOPOFF: settle done, pulse %u/%u dispensed=%luug remain=%luug\n",
                       (unsigned)s_spawn.topoff_pulses, (unsigned)SPAWN_TOPOFF_MAX_PULSES,
                       (unsigned long)s_spawn.dispensed_ug, (unsigned long)remaining_ug);
            }
            return true;
        }

        /* Issue an open nudge if not already nudging */
        if (!s_spawn.nudging)
        {
            s_spawn.topoff_pulses++;
            spawn_set_flap_pwm(true, (uint16_t)SPAWN_OPEN_PWM);
            s_spawn.nudge_until = delayed_by_ms(get_absolute_time(),
                                                (uint32_t)SPAWN_TOPOFF_PULSE_MS);
            s_spawn.nudging = true;
            printf("Spawn TOPOFF: open pulse %u/%u (%ums)\n",
                   (unsigned)s_spawn.topoff_pulses, (unsigned)SPAWN_TOPOFF_MAX_PULSES,
                   (unsigned)SPAWN_TOPOFF_PULSE_MS);
        }
        else
        {
            /* Nudge expired on this tick — stop motor and start settle.
             * Must disable current monitoring to prevent false undercurrent flags
             * when the motor is intentionally stopped (zero current is normal). */
            if (s_drv8263.monitoring_enabled)
            {
                drv8263_stop_current_monitoring(&s_drv8263);
            }
            (void)drv8263_set_motor_control(&s_drv8263, DRV8263_MOTOR_STOP, 0u);
            
            if (s_drv8263_flap2_ready)
            {
                if (s_drv8263_flap2.monitoring_enabled)
                {
                    drv8263_stop_current_monitoring(&s_drv8263_flap2);
                }
                (void)drv8263_set_motor_control(&s_drv8263_flap2, DRV8263_MOTOR_STOP, 0u);
            }
            s_spawn.nudging         = false;
            s_spawn.topoff_settling = true;
            s_spawn.topoff_settle_until = delayed_by_ms(get_absolute_time(),
                                                         (uint32_t)SPAWN_TOPOFF_SETTLE_MS);
        }
        return true;
    }

    default:
        /* Should not reach here — abort safely */
        printf("Spawn: unexpected state %u — aborting\n", (unsigned)s_spawn.state);
        spawn_close_flaps();
        s_spawn.active = false;
        spawn_set_state(SPAWN_SM_FAULT);
        send_spawn_status(SPAWN_STATUS_ERROR);
        return false;
    }
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
    /* busy_wait_us_32, NOT sleep_us — this callback is invoked from the
     * repeating-timer alarm IRQ context.  sleep_us calls __wfe() internally
     * which deadlocks when called from the same alarm IRQ it relies on. */
    busy_wait_us_32(us);
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
        uint32_t now_us = time_us_32();
        uint32_t dt_us = now_us - s_vacuum_last_pulse_time_us;

        if ((s_vacuum_last_pulse_time_us != 0u) && (dt_us < VACUUM_RPM_DEBOUNCE_US))
        {
            return;
        }

        if (s_vacuum_last_pulse_time_us != 0u)
        {
            s_vacuum_last_period_us = dt_us;
            s_vacuum_period_sum_us += dt_us;
            ++s_vacuum_period_samples;
        }

        s_vacuum_last_pulse_time_us = now_us;
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

static bool arm_homing_active(void)
{
    return s_arm_homing_phase != ARM_HOME_IDLE;
}

static void reset_arm_press_retry(void)
{
    memset(&s_arm_press_retry, 0, sizeof(s_arm_press_retry));
    s_arm_press_retry.phase = ARM_PRESS_RETRY_IDLE;
}

static bool arm_press_retry_active(void)
{
    return s_arm_press_retry.phase != ARM_PRESS_RETRY_IDLE;
}

static bool arm_stepper_control_locked(void)
{
    return arm_homing_active() || arm_press_retry_active();
}

static bool arm_motion_requires_rehome(motion_result_t result)
{
    return result == MOTION_STALLED ||
           result == MOTION_TIMEOUT ||
           result == MOTION_FAULT ||
           result == MOTION_SPI_FAULT;
}

static void finish_arm_motion(motion_result_t result, bool require_rehome)
{
    if (require_rehome)
    {
        s_arm_rehome_required = true;
        s_arm_homed = false;
    }

    if (arm_press_retry_active())
    {
        reset_arm_press_retry();
    }

    send_motion_done(SUBSYS_ARM, result, s_arm_pos_steps);
    printf("Arm motion done: result=%u rehome=%u pos=%li\n",
           (unsigned)result, (unsigned)s_arm_rehome_required,
           (long)s_arm_pos_steps);
}

static bool arm_press_retry_should_arm(int32_t target_steps)
{
    return target_steps == (int32_t)ARM_STEPS_PRESS &&
           s_vacuum_on &&
           s_vacuum_rpm_valid &&
           (uint32_t)ARM_PRESS_RETRY_MAX_RETRIES > 0u &&
           (uint32_t)ARM_PRESS_RETRY_BACKOFF_STEPS > 0u &&
           (uint32_t)ARM_PRESS_RETRY_VERIFY_TIMEOUT_MS > 0u;
}

static void set_motion_torque_sample_div(uint8_t sample_div)
{
    s_motion.torque_sample_div = (sample_div == 0u) ? 1u : sample_div;
    s_motion.torque_tick_count = 0u;
}

static bool step_timer_callback(struct repeating_timer *t)
{
    (void)t;
    drv8434s_motion_tick(&s_motion);

    /* Return true to keep the timer running; false to cancel. */
    if (!drv8434s_motion_is_busy(&s_motion))
    {
        s_step_timer_active = false;
        s_step_timer_period_us = 0u;
        return false;
    }
    return true;
}

/* ── Internal helper: ensure step timer is running ──────────────────────────── */

static bool ensure_step_timer_running(uint32_t step_delay_us)
{
    if (step_delay_us == 0u)
    {
        step_delay_us = 1u;
    }

    if (s_step_timer_active)
    {
        return s_step_timer_period_us == step_delay_us;
    }
    if (!add_repeating_timer_us(-(int64_t)step_delay_us,
                                step_timer_callback, NULL, &s_step_timer))
    {
        return false;
    }
    s_step_timer_active = true;
    s_step_timer_period_us = step_delay_us;
    return true;
}

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  Polling tick functions (called from uart_server_poll)                      */
/* ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Periodic idle SPI health check for the DRV8434S chain.
 *
 * When no motion is active, reads the FAULT register from each device every
 * DRV8434S_SPI_WATCHDOG_INTERVAL_MS milliseconds.  A read failure logs a
 * warning.  An active FAULT flag triggers a best-effort clear-faults attempt.
 * This catches persistent SPI issues early, before the next motion command.
 */
static void stepper_spi_watchdog_tick(void)
{
    if (!s_stepper_ready)
        return;

    /* Skip watchdog if any motion is currently in progress */
    if (drv8434s_motion_is_busy(&s_motion))
    {
        /* Reset timer so the watchdog runs after motion completes */
        s_spi_watchdog_last_ms = to_ms_since_boot(get_absolute_time());
        return;
    }

    uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    if ((now_ms - s_spi_watchdog_last_ms) < (uint32_t)DRV8434S_SPI_WATCHDOG_INTERVAL_MS)
        return;

    s_spi_watchdog_last_ms = now_ms;

    for (uint8_t k = 0u; k < (uint8_t)DRV8434S_N_DEVICES; ++k)
    {
        uint8_t fault_val = 0u;
        if (!drv8434s_chain_read_reg(&s_stepper_chain, k,
                                     DRV8434S_REG_FAULT, &fault_val, NULL))
        {
            printf("WARN: DRV8434S dev %u SPI watchdog read FAILED\n", (unsigned)k);
            /* Attempt recovery */
            (void)drv8434s_chain_clear_faults(&s_stepper_chain, k, NULL);
        }
        else if (fault_val != 0u)
        {
            printf("WARN: DRV8434S dev %u idle FAULT=0x%02X — clearing\n",
                   (unsigned)k, (unsigned)fault_val);
            (void)drv8434s_chain_clear_faults(&s_stepper_chain, k, NULL);
        }
    }
}

static motion_result_t motion_result_from_stop_reason(drv8434s_motion_stop_reason_t reason)
{
    if (reason == DRV8434S_MOTION_OK)
    {
        return MOTION_OK;
    }
    if (reason == DRV8434S_MOTION_TORQUE_LIMIT)
    {
        return MOTION_STALLED;
    }
    if (reason == DRV8434S_MOTION_SPI_ERROR)
    {
        return MOTION_SPI_FAULT;
    }
    return MOTION_FAULT;
}

static motion_result_t consume_stepper_motion_result(uint8_t dev_idx, bool timeout,
                                                     int32_t *steps_achieved_out)
{
    *steps_achieved_out = 0;

    if (timeout)
    {
        drv8434s_motion_result_t cancelled;
        if (drv8434s_motion_cancel(&s_motion, dev_idx, &cancelled))
        {
            *steps_achieved_out = cancelled.steps_achieved;
        }
        return MOTION_TIMEOUT;
    }

    *steps_achieved_out = s_motion.jobs[dev_idx].steps_achieved;
    return motion_result_from_stop_reason(s_motion.jobs[dev_idx].reason);
}

/**
 * @brief Flap state-machine tick — detects current threshold crossings.
 *
 * The DRV8263 monitoring timer updates s_drv8263.current_status in ISR
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
            s_drv8263.current_status == DRV8263_CURRENT_UNDERCURRENT)
        {
            drv8263_stop_current_monitoring(&s_drv8263);
            (void)drv8263_set_motor_control(&s_drv8263, DRV8263_MOTOR_STOP, 0u);
            s_flap1_stopped = true;
            printf("Flap1 OPEN: reached endpoint (current drop)\n");
        }

        if (!s_flap2_stopped && s_drv8263_flap2_ready &&
            s_drv8263_flap2.current_status == DRV8263_CURRENT_UNDERCURRENT)
        {
            drv8263_stop_current_monitoring(&s_drv8263_flap2);
            (void)drv8263_set_motor_control(&s_drv8263_flap2, DRV8263_MOTOR_STOP, 0u);
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
                drv8263_stop_current_monitoring(&s_drv8263);
                (void)drv8263_set_motor_control(&s_drv8263, DRV8263_MOTOR_STOP, 0u);
            }
            if (!s_flap2_stopped && s_drv8263_flap2_ready)
            {
                drv8263_stop_current_monitoring(&s_drv8263_flap2);
                (void)drv8263_set_motor_control(&s_drv8263_flap2, DRV8263_MOTOR_STOP, 0u);
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
                       (s_drv8263.current_status == DRV8263_CURRENT_OVERCURRENT ||
                        s_drv8263.current_status == DRV8263_CURRENT_UNDERCURRENT);
        bool f2_done = !s_flap2_stopped && s_drv8263_flap2_ready &&
                       (s_drv8263_flap2.current_status == DRV8263_CURRENT_OVERCURRENT ||
                        s_drv8263_flap2.current_status == DRV8263_CURRENT_UNDERCURRENT);

        /* Either flap reaching its endpoint stops both immediately */
        if (f1_done || f2_done)
        {
            if (!s_flap1_stopped)
            {
                drv8263_stop_current_monitoring(&s_drv8263);
                (void)drv8263_set_motor_control(&s_drv8263, DRV8263_MOTOR_STOP, 0u);
                s_flap1_stopped = true;
                printf("Flap1 CLOSE: stopped (%s)\n",
                       f1_done
                           ? (s_drv8263.current_status == DRV8263_CURRENT_UNDERCURRENT
                                  ? "undercurrent"
                                  : "torque")
                           : "partner endpoint");
            }
            if (!s_flap2_stopped && s_drv8263_flap2_ready)
            {
                drv8263_stop_current_monitoring(&s_drv8263_flap2);
                (void)drv8263_set_motor_control(&s_drv8263_flap2, DRV8263_MOTOR_STOP, 0u);
                s_flap2_stopped = true;
                printf("Flap2 CLOSE: stopped (%s)\n",
                       f2_done
                           ? (s_drv8263_flap2.current_status == DRV8263_CURRENT_UNDERCURRENT
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
                drv8263_stop_current_monitoring(&s_drv8263);
                (void)drv8263_set_motor_control(&s_drv8263, DRV8263_MOTOR_STOP, 0u);
            }
            if (!s_flap2_stopped && s_drv8263_flap2_ready)
            {
                drv8263_stop_current_monitoring(&s_drv8263_flap2);
                (void)drv8263_set_motor_control(&s_drv8263_flap2, DRV8263_MOTOR_STOP, 0u);
            }
            printf("Flap CLOSE: timeout\n");
            s_flap_state = FLAP_SM_FAULT;
            send_motion_done(SUBSYS_FLAPS, MOTION_TIMEOUT, 0);
        }
    }
}

static void finish_arm_home(motion_result_t result)
{
    reset_arm_press_retry();
    set_motion_torque_sample_div((uint8_t)STEPPER_DEFAULT_TORQUE_SAMPLE_DIV);
    s_arm_homing_phase = ARM_HOME_IDLE;
    s_arm_pending.pending = false;
    s_arm_homed = (result == MOTION_OK);
    s_arm_rehome_required = (result != MOTION_OK);
    send_motion_done(SUBSYS_ARM, result, s_arm_pos_steps);
    printf("Arm home done: result=%u homed=%u pos=%li\n",
           (unsigned)result, (unsigned)s_arm_homed, (long)s_arm_pos_steps);
}

static void handle_arm_home_completion(motion_result_t result,
                                       int32_t steps_achieved)
{
    if (s_arm_homing_phase == ARM_HOME_SEEK)
    {
        if (result == MOTION_STALLED)
        {
            s_arm_pos_steps = 0;
            printf("Arm home: hard stop found, backing off %u steps\n",
                   (unsigned)ARM_HOME_BACKOFF_STEPS);
            if (start_arm_home_backoff())
            {
                return;
            }
            finish_arm_home(MOTION_FAULT);
            return;
        }

        if (result == MOTION_OK)
        {
            result = MOTION_TIMEOUT;
        }

        (void)steps_achieved;
        finish_arm_home(result);
        return;
    }

    if (s_arm_homing_phase == ARM_HOME_BACKOFF)
    {
        s_arm_pos_steps += steps_achieved;
        finish_arm_home(result);
        return;
    }

    s_arm_pending.pending = false;
    send_motion_done(SUBSYS_ARM, result, s_arm_pos_steps);
}

static bool arm_press_stall_is_success(const stepper_pending_t *pending,
                                       motion_result_t result)
{
    return pending != NULL &&
           pending->target_steps == (int32_t)ARM_STEPS_PRESS &&
           result == MOTION_STALLED;
}

static bool handle_arm_press_retry_completion(motion_result_t result,
                                              uint32_t now_ms)
{
    if (!arm_press_retry_active())
    {
        return false;
    }

    switch (s_arm_press_retry.phase)
    {
    case ARM_PRESS_RETRY_PRESSING:
    case ARM_PRESS_RETRY_REPRESSING:
        if (result != MOTION_OK)
        {
            finish_arm_motion(result, arm_motion_requires_rehome(result));
            return true;
        }

        s_arm_press_retry.phase = ARM_PRESS_RETRY_VERIFY;
        s_arm_press_retry.phase_start_ms = now_ms;
        printf("Arm press verify: baseline RPM=%u retry=%u/%u\n",
               (unsigned)s_arm_press_retry.baseline_rpm,
               (unsigned)s_arm_press_retry.retries_started,
               (unsigned)ARM_PRESS_RETRY_MAX_RETRIES);
        return true;

    case ARM_PRESS_RETRY_BACKOFF:
        if (result != MOTION_OK)
        {
            finish_arm_motion(result, arm_motion_requires_rehome(result));
            return true;
        }

        s_arm_press_retry.phase = ARM_PRESS_RETRY_REPRESSING;
        s_arm_press_retry.baseline_rpm = s_vacuum_last_rpm;
        s_arm_press_retry.phase_start_ms = now_ms;

        if (!start_arm_internal_absolute_move((int32_t)ARM_STEPS_PRESS,
                                              (uint16_t)STEPPER_SOFT_TORQUE_LIMIT,
                                              (uint32_t)ARM_MOTION_TIMEOUT_MS))
        {
            finish_arm_motion(MOTION_FAULT, false);
            return true;
        }

        printf("Arm press retry: re-pressing from RPM=%u\n",
               (unsigned)s_arm_press_retry.baseline_rpm);
        return true;

    case ARM_PRESS_RETRY_VERIFY:
    case ARM_PRESS_RETRY_IDLE:
    default:
        return false;
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

    /* ── ARM — device STEPPER_DEV_ROT_ARM ───────────────────── */
    if (s_arm_pending.pending)
    {
        bool job_done = !s_motion.jobs[STEPPER_DEV_ROT_ARM].active;
        bool timeout = (now_ms - s_arm_pending.start_ms) >= s_arm_pending.timeout_ms;

        if (job_done || timeout)
        {
            int32_t steps_achieved = 0;
            motion_result_t result =
                consume_stepper_motion_result((uint8_t)STEPPER_DEV_ROT_ARM,
                                              timeout && !job_done,
                                              &steps_achieved);

            if (arm_homing_active())
            {
                handle_arm_home_completion(result, steps_achieved);
            }
            else
            {
                bool press_target = (s_arm_pending.target_steps == (int32_t)ARM_STEPS_PRESS);
                s_arm_pos_steps += steps_achieved;
                s_arm_pending.pending = false;

                if (arm_press_stall_is_success(&s_arm_pending, result))
                {
                    printf("Arm press: torque stop treated as success at pos=%li\n",
                           (long)s_arm_pos_steps);
                    result = MOTION_OK;
                }

                if (handle_arm_press_retry_completion(result, now_ms))
                {
                    return;
                }

                if (press_target && arm_press_retry_should_arm(s_arm_pending.target_steps))
                {
                    s_arm_press_retry.phase = ARM_PRESS_RETRY_VERIFY;
                    s_arm_press_retry.retries_started = 0u;
                    s_arm_press_retry.baseline_rpm = s_vacuum_last_rpm;
                    s_arm_press_retry.phase_start_ms = now_ms;
                    printf("Arm press verify: baseline RPM=%u retry=%u/%u\n",
                           (unsigned)s_arm_press_retry.baseline_rpm,
                           (unsigned)s_arm_press_retry.retries_started,
                           (unsigned)ARM_PRESS_RETRY_MAX_RETRIES);
                    return;
                }

                finish_arm_motion(result, arm_motion_requires_rehome(result));
            }
        }
    }

#ifdef STEPPER_DEV_LIN_ARM
    /* ── RACK (linear arm) — device STEPPER_DEV_LIN_ARM ────────── */
    if (s_rack_pending.pending)
    {
        bool job_done = !s_motion.jobs[STEPPER_DEV_LIN_ARM].active;
        bool timeout = (now_ms - s_rack_pending.start_ms) >= s_rack_pending.timeout_ms;

        if (job_done || timeout)
        {
            int32_t steps_achieved = 0;
            motion_result_t result =
                consume_stepper_motion_result((uint8_t)STEPPER_DEV_LIN_ARM,
                                              timeout && !job_done,
                                              &steps_achieved);

            s_rack_pos_steps += steps_achieved;

            send_motion_done(SUBSYS_RACK, result, s_rack_pos_steps);
            s_rack_pending.pending = false;
            printf("Rack motion done: result=%u pos=%li\n",
                   (unsigned)result, (long)s_rack_pos_steps);
        }
    }
#endif /* STEPPER_DEV_LIN_ARM */

    /* ── TURNTABLE — device STEPPER_DEV_TURNTABLE ───────────── */
    if (s_turntable_pending.pending)
    {
        bool job_done = !s_motion.jobs[STEPPER_DEV_TURNTABLE].active;
        bool timeout = (now_ms - s_turntable_pending.start_ms) >= s_turntable_pending.timeout_ms;

        if (job_done || timeout)
        {
            int32_t steps_achieved = 0;
            motion_result_t result =
                consume_stepper_motion_result((uint8_t)STEPPER_DEV_TURNTABLE,
                                              timeout && !job_done,
                                              &steps_achieved);

            s_turntable_pos_steps += steps_achieved;

            send_motion_done(SUBSYS_TURNTABLE, result, s_turntable_pos_steps);
            s_turntable_pending.pending = false;
            printf("Turntable motion done: result=%u pos=%li\n",
                   (unsigned)result, (long)s_turntable_pos_steps);
        }
    }

#ifdef STEPPER_DEV_HW_CARRIAGE
    /* ── HOTWIRE CARRIAGE — device STEPPER_DEV_HW_CARRIAGE ───────── */
    if (s_hotwire_pending.pending)
    {
        bool job_done = !s_motion.jobs[STEPPER_DEV_HW_CARRIAGE].active;
        bool timeout = (now_ms - s_hotwire_pending.start_ms) >= s_hotwire_pending.timeout_ms;

        if (job_done || timeout)
        {
            int32_t steps_achieved = 0;
            motion_result_t result =
                consume_stepper_motion_result((uint8_t)STEPPER_DEV_HW_CARRIAGE,
                                              timeout && !job_done,
                                              &steps_achieved);

            s_hotwire_pos_steps += steps_achieved;
            send_motion_done(SUBSYS_HOTWIRE, result, s_hotwire_pos_steps);
            s_hotwire_pending.pending = false;
            printf("Hotwire traverse done: result=%u pos=%li\n",
                   (unsigned)result, (long)s_hotwire_pos_steps);
        }
    }
#endif /* STEPPER_DEV_HW_CARRIAGE */

#ifdef STEPPER_DEV_AGITATOR
    /* ── AGITATOR — multi-phase: homing → backoff → N×(forward+reverse) ─── */
    if (s_agitator_pending.pending)
    {
        bool job_done = !s_motion.jobs[STEPPER_DEV_AGITATOR].active;
        bool timeout  = (now_ms - s_agitator_pending.start_ms) >= s_agitator_pending.timeout_ms;

        if (job_done || timeout)
        {
            int32_t steps_achieved = 0;
            motion_result_t result =
                consume_stepper_motion_result((uint8_t)STEPPER_DEV_AGITATOR,
                                              timeout && !job_done,
                                              &steps_achieved);

            /* ── HOMING phase: stall = home found; end of steps = already homed ─ */
            if (s_agitator_phase == AGIT_PHASE_HOMING)
            {
                /* Restore normal torque sample divisor after sensitive homing mode */
                set_motion_torque_sample_div((uint8_t)STEPPER_DEFAULT_TORQUE_SAMPLE_DIV);

                /* Either a torque stall (home endstop) or exhausted search steps.
                 * Both are acceptable home conditions — zero the position. */
                s_agitator_pos_steps = 0;
                printf("Agitator homing: endstop reached (result=%u steps=%li) — backing off %u steps\n",
                       (unsigned)result, (long)steps_achieved,
                       (unsigned)AGITATOR_HOME_BACKOFF_STEPS);
                s_agitator_phase = AGIT_PHASE_HOMING_BACKOFF;
                if (start_stepper_job((uint8_t)STEPPER_DEV_AGITATOR,
                                      -(int32_t)AGITATOR_HOME_BACKOFF_STEPS,
                                      0u, 0u,
                                      (uint32_t)AGITATOR_STEP_DELAY_US))
                {
                    s_agitator_pending.target_steps = -(int32_t)AGITATOR_HOME_BACKOFF_STEPS;
                    s_agitator_pending.start_ms     = now_ms;
                }
                else
                {
                    /* Can't backoff — follow home-only or cycle behavior */
                    printf("Agitator homing: backoff start failed\n");
                    if (s_agitator_home_only)
                    {
                        s_agitator_pending.pending = false;
                        s_agitator_phase           = AGIT_PHASE_IDLE;
                        s_agitator_cycle_done      = true;
                        s_agitator_home_only       = false;
                        send_motion_done(SUBSYS_AGITATOR, MOTION_OK, s_agitator_pos_steps);
                    }
                    else
                    {
                        s_agitator_phase = AGIT_PHASE_FORWARD;
                        if (!start_stepper_job((uint8_t)STEPPER_DEV_AGITATOR,
                                               (int32_t)AGITATOR_KNEAD_STEPS,
                                               0u, 0u,
                                               (uint32_t)AGITATOR_STEP_DELAY_US))
                        {
                            /* Nothing works — abort */
                            s_agitator_pending.pending = false;
                            s_agitator_phase           = AGIT_PHASE_IDLE;
                            s_agitator_cycle_done      = true;
                            send_motion_done(SUBSYS_AGITATOR, MOTION_FAULT, 0);
                        }
                        else
                        {
                            s_agitator_pending.target_steps = (int32_t)AGITATOR_KNEAD_STEPS;
                            s_agitator_pending.start_ms     = now_ms;
                        }
                    }
                }
            }
            /* ── HOMING_BACKOFF phase: position released from endstop ─────────── */
            else if (s_agitator_phase == AGIT_PHASE_HOMING_BACKOFF)
            {
                s_agitator_pos_steps += steps_achieved; /* negative — moving away from stop */

                if (s_agitator_home_only)
                {
                    printf("Agitator homing complete (home-only, pos=%li)\n",
                           (long)s_agitator_pos_steps);
                    s_agitator_pending.pending = false;
                    s_agitator_phase           = AGIT_PHASE_IDLE;
                    s_agitator_cycle_done      = true;
                    s_agitator_home_only       = false;
                    send_motion_done(SUBSYS_AGITATOR, MOTION_OK, s_agitator_pos_steps);
                }
                else
                {
                    printf("Agitator backoff done (pos=%li) — starting %u-cycle knead\n",
                           (long)s_agitator_pos_steps, (unsigned)s_agitator_n_cycles);
                    s_agitator_cycles_done = 0u;
                    s_agitator_phase = AGIT_PHASE_FORWARD;
                    if (!start_stepper_job((uint8_t)STEPPER_DEV_AGITATOR,
                                           (int32_t)AGITATOR_KNEAD_STEPS,
                                           0u, 0u,
                                           (uint32_t)AGITATOR_STEP_DELAY_US))
                    {
                        s_agitator_pending.pending = false;
                        s_agitator_phase           = AGIT_PHASE_IDLE;
                        s_agitator_cycle_done      = true;
                        send_motion_done(SUBSYS_AGITATOR, MOTION_FAULT, 0);
                    }
                    else
                    {
                        s_agitator_pending.target_steps = (int32_t)AGITATOR_KNEAD_STEPS;
                        s_agitator_pending.start_ms     = now_ms;
                    }
                }
            }
            /* ── FORWARD stroke: queue reverse ───────────────────────────────── */
            else if (s_agitator_phase == AGIT_PHASE_FORWARD && !timeout && result == MOTION_OK)
            {
                s_agitator_pos_steps += steps_achieved;
                printf("Agitator: fwd done (cycle %u/%u pos=%li) — reversing\n",
                       (unsigned)(s_agitator_cycles_done + 1u),
                       (unsigned)s_agitator_n_cycles,
                       (long)s_agitator_pos_steps);
                s_agitator_phase = AGIT_PHASE_REVERSE;
                if (start_stepper_job((uint8_t)STEPPER_DEV_AGITATOR,
                                      -(int32_t)AGITATOR_KNEAD_STEPS,
                                      0u, 0u,
                                      (uint32_t)AGITATOR_STEP_DELAY_US))
                {
                    s_agitator_pending.target_steps = -(int32_t)AGITATOR_KNEAD_STEPS;
                    s_agitator_pending.start_ms     = now_ms;
                }
                else
                {
                    /* Reverse failed — count as done */
                    printf("Agitator: reverse start failed — ending early\n");
                    s_agitator_cycles_done++;
                    s_agitator_pending.pending = false;
                    s_agitator_phase           = AGIT_PHASE_IDLE;
                    s_agitator_cycle_done      = true;
                    send_motion_done(SUBSYS_AGITATOR, MOTION_OK, s_agitator_pos_steps);
                }
            }
            /* ── REVERSE stroke: count cycle; repeat or finish ─────────────────── */
            else if (s_agitator_phase == AGIT_PHASE_REVERSE && !timeout && result == MOTION_OK)
            {
                s_agitator_pos_steps += steps_achieved;
                s_agitator_cycles_done++;
                printf("Agitator: rev done — %u/%u cycles complete (pos=%li)\n",
                       (unsigned)s_agitator_cycles_done,
                       (unsigned)s_agitator_n_cycles,
                       (long)s_agitator_pos_steps);

                if (s_agitator_cycles_done < s_agitator_n_cycles)
                {
                    /* Start next forward stroke */
                    s_agitator_phase = AGIT_PHASE_FORWARD;
                    if (start_stepper_job((uint8_t)STEPPER_DEV_AGITATOR,
                                          (int32_t)AGITATOR_KNEAD_STEPS,
                                          0u, 0u,
                                          (uint32_t)AGITATOR_STEP_DELAY_US))
                    {
                        s_agitator_pending.target_steps = (int32_t)AGITATOR_KNEAD_STEPS;
                        s_agitator_pending.start_ms     = now_ms;
                    }
                    else
                    {
                        printf("Agitator: next cycle start failed\n");
                        s_agitator_pending.pending = false;
                        s_agitator_phase           = AGIT_PHASE_IDLE;
                        s_agitator_cycle_done      = true;
                        send_motion_done(SUBSYS_AGITATOR, MOTION_FAULT, s_agitator_pos_steps);
                    }
                }
                else
                {
                    /* All cycles complete */
                    s_agitator_pending.pending = false;
                    s_agitator_phase           = AGIT_PHASE_IDLE;
                    s_agitator_cycle_done      = true;
                    printf("Agitator: all %u cycles done (pos=%li)\n",
                           (unsigned)s_agitator_n_cycles, (long)s_agitator_pos_steps);
                    send_motion_done(SUBSYS_AGITATOR, MOTION_OK, s_agitator_pos_steps);
                }
            }
            /* ── Timeout or fault on any phase ──────────────────────────────────── */
            else
            {
                printf("Agitator: timeout/fault in phase %u (result=%u steps=%li)\n",
                       (unsigned)s_agitator_phase, (unsigned)result, (long)steps_achieved);
                s_agitator_pending.pending = false;
                s_agitator_phase           = AGIT_PHASE_IDLE;
                s_agitator_cycle_done      = true;
                s_agitator_home_only       = false;
                send_motion_done(SUBSYS_AGITATOR, result, steps_achieved);
            }
        }
    }
#endif /* STEPPER_DEV_AGITATOR */
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
        /* Atomically snapshot period statistics accumulated by the ISR. */
        uint32_t saved = save_and_disable_interrupts();
        uint32_t pulses = s_vacuum_pulse_count;
        uint32_t last_pulse_time_us = s_vacuum_last_pulse_time_us;
        uint32_t last_period_us = s_vacuum_last_period_us;
        uint64_t period_sum_us = s_vacuum_period_sum_us;
        uint32_t period_samples = s_vacuum_period_samples;
        s_vacuum_pulse_count = 0u;
        s_vacuum_period_sum_us = 0u;
        s_vacuum_period_samples = 0u;
        restore_interrupts(saved);

        uint16_t rpm = 0u;
        bool rpm_valid = false;

        if (last_pulse_time_us != 0u)
        {
            uint32_t pulse_age_ms = (time_us_32() - last_pulse_time_us) / 1000u;
            if (pulse_age_ms < VACUUM_RPM_TIMEOUT_MS)
            {
                uint32_t avg_period_us = 0u;
                if (period_samples > 0u)
                {
                    avg_period_us = (uint32_t)(period_sum_us / period_samples);
                }
                else if (last_period_us > 0u)
                {
                    avg_period_us = last_period_us;
                }

                if (avg_period_us > 0u)
                {
                    uint32_t rpm32 = (uint32_t)(60000000ull /
                                                ((uint64_t)avg_period_us *
                                                 (uint64_t)VACUUM_PULSES_PER_REV));
                    rpm = (rpm32 > UINT16_MAX) ? UINT16_MAX : (uint16_t)rpm32;
                    rpm_valid = true;
                }
            }
        }

        s_vacuum_last_rpm = rpm;
        s_vacuum_rpm_valid = rpm_valid;

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

static bool start_arm_internal_absolute_move(int32_t target_steps,
                                             uint16_t torque_limit,
                                             uint32_t timeout_ms)
{
    int32_t delta = target_steps - s_arm_pos_steps;
    if (delta == 0)
    {
        s_arm_pending.pending = false;
        s_arm_pending.target_steps = target_steps;
        return true;
    }

    return start_arm_internal_relative_move(delta, torque_limit, timeout_ms);
}

static bool start_arm_internal_relative_move(int32_t relative_steps,
                                             uint16_t torque_limit,
                                             uint32_t timeout_ms)
{
    int32_t target_steps = s_arm_pos_steps + relative_steps;

    if (!start_stepper_job((uint8_t)STEPPER_DEV_ROT_ARM,
                           relative_steps,
                           torque_limit,
                           (uint16_t)DRV8434S_MOTION_TORQUE_BLANK_STEPS,
                           (uint32_t)STEPPER_DEFAULT_STEP_DELAY_US))
    {
        return false;
    }

    s_arm_pending.pending = true;
    s_arm_pending.target_steps = target_steps;
    s_arm_pending.subsys = SUBSYS_ARM;
    s_arm_pending.start_ms = to_ms_since_boot(get_absolute_time());
    s_arm_pending.timeout_ms = timeout_ms;
    s_arm_pending.dev_idx = (uint8_t)STEPPER_DEV_ROT_ARM;
    return true;
}

static void arm_press_retry_tick(void)
{
    if (s_arm_press_retry.phase != ARM_PRESS_RETRY_VERIFY)
    {
        return;
    }

    uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    uint16_t current_rpm = s_vacuum_last_rpm;
    uint16_t baseline_rpm = s_arm_press_retry.baseline_rpm;
    uint16_t rpm_delta = (current_rpm >= baseline_rpm)
                             ? (uint16_t)(current_rpm - baseline_rpm)
                             : (uint16_t)(baseline_rpm - current_rpm);

    if (!s_vacuum_on || !s_vacuum_rpm_valid)
    {
        printf("Arm press verify: vacuum RPM unavailable, accepting press\n");
        finish_arm_motion(MOTION_OK, false);
        return;
    }

    if (rpm_delta >= (uint16_t)ARM_PRESS_RETRY_RPM_DELTA)
    {
        printf("Arm press verify: RPM delta=%u (%u -> %u), press accepted\n",
               (unsigned)rpm_delta, (unsigned)baseline_rpm, (unsigned)current_rpm);
        finish_arm_motion(MOTION_OK, false);
        return;
    }

    if ((now_ms - s_arm_press_retry.phase_start_ms) <
        (uint32_t)ARM_PRESS_RETRY_VERIFY_TIMEOUT_MS)
    {
        return;
    }

    if (s_arm_press_retry.retries_started >= (uint8_t)ARM_PRESS_RETRY_MAX_RETRIES)
    {
        printf("Arm press verify: no RPM change after %u retry(ies), reporting timeout\n",
               (unsigned)s_arm_press_retry.retries_started);
        finish_arm_motion(MOTION_TIMEOUT, false);
        return;
    }

    ++s_arm_press_retry.retries_started;
    s_arm_press_retry.phase = ARM_PRESS_RETRY_BACKOFF;
    s_arm_press_retry.phase_start_ms = now_ms;

    if (!start_arm_internal_relative_move((int32_t)ARM_PRESS_RETRY_BACKOFF_STEPS,
                                          0u,
                                          (uint32_t)ARM_MOTION_TIMEOUT_MS))
    {
        finish_arm_motion(MOTION_FAULT, false);
        return;
    }

    printf("Arm press verify: RPM delta=%u, retry %u/%u with backoff=%u steps\n",
           (unsigned)rpm_delta,
           (unsigned)s_arm_press_retry.retries_started,
           (unsigned)ARM_PRESS_RETRY_MAX_RETRIES,
           (unsigned)ARM_PRESS_RETRY_BACKOFF_STEPS);
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

    if (arm_stepper_control_locked())
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
    const uint16_t expected_new = (uint16_t)sizeof(pl_stepper_stepjob_t);
    const uint16_t expected_old = (uint16_t)(sizeof(pl_stepper_stepjob_t) - sizeof(uint16_t));

    if (len != expected_new && len != expected_old)
    {
        send_nack(seq, NACK_BAD_LEN);
        return;
    }

    if (!s_stepper_ready)
    {
        send_nack(seq, NACK_UNKNOWN);
        return;
    }

    if (arm_stepper_control_locked())
    {
        send_nack(seq, NACK_UNKNOWN);
        return;
    }

    const pl_stepper_stepjob_t *p = (const pl_stepper_stepjob_t *)payload;
    uint16_t torque_limit = 0u;
    if (len == expected_new)
    {
        torque_limit = p->torque_limit;
    }

    int32_t target = (p->dir == 0) ? (int32_t)p->steps : -(int32_t)p->steps;

        printf("Stepper: %lu steps dir=%u delay=%lu us torque_limit=%u\n",
           (unsigned long)p->steps,
           (unsigned)p->dir,
           (unsigned long)p->step_delay_us,
           (unsigned)torque_limit);

    /* Start motion on device 0 (non-blocking) */
    if (!start_stepper_job(0u, target, torque_limit,
                           (uint16_t)DRV8434S_MOTION_TORQUE_BLANK_STEPS,
                           p->step_delay_us))
    {
        send_nack(seq, NACK_UNKNOWN);
        return;
    }

    send_ack(seq);
}

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  Existing low-level DRV8263 handlers (unchanged)                            */
/* ═══════════════════════════════════════════════════════════════════════════ */

static void handle_drv8263_start(uint16_t seq, const uint8_t *payload, uint16_t len)
{
    if (len != (uint16_t)sizeof(pl_drv8263_start_mon_t))
    {
        send_nack(seq, NACK_BAD_LEN);
        return;
    }

    if (!s_drv8263_ready)
    {
        send_nack(seq, NACK_UNKNOWN);
        return;
    }

    const pl_drv8263_start_mon_t *p = (const pl_drv8263_start_mon_t *)payload;

    s_drv8263.config.current_low_threshold = p->low_th;
    s_drv8263.config.current_high_threshold = p->high_th;
    s_drv8263.config.current_check_interval_ms = p->interval_ms;

    drv8263_motor_state_t st = (p->dir == 0) ? DRV8263_MOTOR_FORWARD : DRV8263_MOTOR_REVERSE;
    (void)drv8263_set_motor_control(&s_drv8263, st, p->speed);

    if (s_drv8263.monitoring_enabled)
    {
        drv8263_stop_current_monitoring(&s_drv8263);
    }

    if (!drv8263_start_current_monitoring(&s_drv8263))
    {
        send_nack(seq, NACK_UNKNOWN);
        return;
    }

    send_ack(seq);
}

static void handle_drv8263_stop(uint16_t seq)
{
    if (!s_drv8263_ready)
    {
        send_nack(seq, NACK_UNKNOWN);
        return;
    }

    drv8263_stop_current_monitoring(&s_drv8263);
    (void)drv8263_set_motor_control(&s_drv8263, DRV8263_MOTOR_STOP, 0u);
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
 * @brief MSG_FLAPS_OPEN (0x40) — drive flap DRV8263 forward; auto-stop on
 *        current drop (open-circuit end-of-travel).
 */
static void handle_flap_open(uint16_t seq)
{
    if (!s_drv8263_ready)
    {
        send_nack(seq, NACK_UNKNOWN);
        return;
    }

    /* No-op: already open — ACK and send immediate MOTION_DONE(OK). */
    if (s_flap_state == FLAP_SM_OPEN)
    {
        printf("Flap OPEN: already open (no-op)\n");
        send_ack(seq);
        send_motion_done(SUBSYS_FLAPS, MOTION_OK, 0);
        return;
    }

    /* FAULT recovery: log and fall through to restart the open sequence. */
    if (s_flap_state == FLAP_SM_FAULT)
    {
        printf("Flap OPEN: clearing fault, restarting open\n");
        /* Fall through — monitoring stop below will clean up any stale state. */
    }

    /* Interrupt-in-progress: if closing, stop and reverse direction. */
    if (s_flap_state == FLAP_SM_CLOSING)
    {
        printf("Flap OPEN: interrupting close (reversing to open)\n");
    }

    /* Stop any in-progress monitoring before reconfiguring. */
    if (s_drv8263.monitoring_enabled)
    {
        drv8263_stop_current_monitoring(&s_drv8263);
    }

    /* Configure: low threshold = current-drop stop; high threshold disabled. */
    s_drv8263.config.current_low_threshold = (uint16_t)FLAP_OPEN_CURRENT_DROP_TH;
    s_drv8263.config.current_high_threshold = 4095u;
    s_drv8263.config.current_check_interval_ms = (uint32_t)FLAP_MONITOR_INTERVAL_MS;

    (void)drv8263_set_motor_control(&s_drv8263, DRV8263_MOTOR_FORWARD,
                                    (uint16_t)FLAP_OPEN_SPEED_PWM);

    if (!drv8263_start_current_monitoring(&s_drv8263))
    {
        (void)drv8263_set_motor_control(&s_drv8263, DRV8263_MOTOR_STOP, 0u);
        send_nack(seq, NACK_UNKNOWN);
        return;
    }

    /* Start flap 2 if available */
    if (s_drv8263_flap2_ready)
    {
        if (s_drv8263_flap2.monitoring_enabled)
        {
            drv8263_stop_current_monitoring(&s_drv8263_flap2);
        }
        s_drv8263_flap2.config.current_low_threshold = (uint16_t)FLAP_OPEN_CURRENT_DROP_TH;
        s_drv8263_flap2.config.current_high_threshold = 4095u;
        s_drv8263_flap2.config.current_check_interval_ms = (uint32_t)FLAP_MONITOR_INTERVAL_MS;
        (void)drv8263_set_motor_control(&s_drv8263_flap2, DRV8263_MOTOR_FORWARD,
                                        (uint16_t)FLAP_OPEN_SPEED_PWM);
        (void)drv8263_start_current_monitoring(&s_drv8263_flap2);
    }

    s_flap_state = FLAP_SM_OPENING;
    s_flap_start_ms = to_ms_since_boot(get_absolute_time());
    s_flap1_stopped = false;
    s_flap2_stopped = (!s_drv8263_flap2_ready);
    printf("Flap OPEN started\n");
    send_ack(seq);
}

/**
 * @brief MSG_FLAPS_CLOSE (0x41) — drive flap DRV8263 reverse; auto-stop on
 *        torque rise (current high = mechanical stop).
 *        Delegates entirely to flap_close_internal() to avoid code duplication
 *        with the spawn dosing path.
 */
static void handle_flap_close(uint16_t seq)
{
    /* No-op: already closed — ACK and send immediate MOTION_DONE(OK). */
    if (s_flap_state == FLAP_SM_CLOSED)
    {
        printf("Flap CLOSE: already closed (no-op)\n");
        send_ack(seq);
        send_motion_done(SUBSYS_FLAPS, MOTION_OK, 0);
        return;
    }

    /* FAULT recovery: log and fall through — flap_close_internal() cleans up. */
    if (s_flap_state == FLAP_SM_FAULT)
    {
        printf("Flap CLOSE: clearing fault, restarting close\n");
    }

    /* Interrupt-in-progress: if opening, stop and reverse direction. */
    if (s_flap_state == FLAP_SM_OPENING)
    {
        printf("Flap CLOSE: interrupting open (reversing to close)\n");
        /* flap_close_internal() stops monitoring before starting close. */
    }

    if (!flap_close_internal())
    {
        send_nack(seq, NACK_UNKNOWN);
        return;
    }
    printf("Flap CLOSE started\n");
    send_ack(seq);
}

/* ── Internal helper: start a stepper motion and arm the pending tracker ───── */

static bool start_stepper_job(uint8_t dev_idx, int32_t relative_steps,
                              uint16_t torque_limit,
                              uint16_t torque_blank_steps,
                              uint32_t step_delay_us)
{
    if (!drv8434s_motion_start_ex(&s_motion, dev_idx, relative_steps,
                                  torque_limit, torque_blank_steps))
    {
        return false;
    }

    if (!ensure_step_timer_running(step_delay_us))
    {
        drv8434s_motion_cancel(&s_motion, dev_idx, NULL);
        return false;
    }

    return true;
}

static bool start_stepper_motion(uint8_t dev_idx, int32_t target_steps,
                                 stepper_pending_t *pending,
                                 subsystem_id_t subsys, uint32_t timeout_ms,
                                 uint16_t seq)
{
    int32_t current_steps;
    if (dev_idx == (uint8_t)STEPPER_DEV_ROT_ARM)
    {
        current_steps = s_arm_pos_steps;
    }
#ifdef STEPPER_DEV_LIN_ARM
    else if (dev_idx == (uint8_t)STEPPER_DEV_LIN_ARM)
    {
        current_steps = s_rack_pos_steps;
    }
#endif
    else if (dev_idx == (uint8_t)STEPPER_DEV_TURNTABLE)
    {
        current_steps = s_turntable_pos_steps;
    }
    else
    {
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

    if (!start_stepper_job(dev_idx, delta,
                           (uint16_t)STEPPER_SOFT_TORQUE_LIMIT,
                           (uint16_t)DRV8434S_MOTION_TORQUE_BLANK_STEPS,
                           (uint32_t)STEPPER_DEFAULT_STEP_DELAY_US))
    {
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

static bool start_arm_home_backoff(void)
{
    if (!start_stepper_job((uint8_t)STEPPER_DEV_ROT_ARM,
                           -(int32_t)ARM_HOME_BACKOFF_STEPS,
                           0u,
                           (uint16_t)DRV8434S_MOTION_TORQUE_BLANK_STEPS,
                           (uint32_t)ARM_HOME_STEP_DELAY_US))
    {
        return false;
    }

    s_arm_homing_phase = ARM_HOME_BACKOFF;
    s_arm_pending.target_steps = -(int32_t)ARM_HOME_BACKOFF_STEPS;
    s_arm_pending.start_ms = to_ms_since_boot(get_absolute_time());
    s_arm_pending.timeout_ms = (uint32_t)ARM_HOME_TIMEOUT_MS;
    s_arm_pending.dev_idx = (uint8_t)STEPPER_DEV_ROT_ARM;
    return true;
}

static void handle_arm_home(uint16_t seq)
{
    if (!s_stepper_ready)
    {
        send_nack(seq, NACK_UNKNOWN);
        return;
    }

    if (arm_stepper_control_locked() || drv8434s_motion_is_busy(&s_motion))
    {
        send_nack(seq, NACK_UNKNOWN);
        return;
    }

    set_motion_torque_sample_div((uint8_t)ARM_HOME_TORQUE_SAMPLE_DIV);
    if (!start_stepper_job((uint8_t)STEPPER_DEV_ROT_ARM,
                           (int32_t)ARM_HOME_SEARCH_STEPS,
                           (uint16_t)ARM_HOME_TORQUE_LIMIT,
                           (uint16_t)ARM_HOME_TORQUE_BLANK_STEPS,
                           (uint32_t)ARM_HOME_STEP_DELAY_US))
    {
        set_motion_torque_sample_div((uint8_t)STEPPER_DEFAULT_TORQUE_SAMPLE_DIV);
        send_nack(seq, NACK_UNKNOWN);
        return;
    }

    s_arm_homing_phase = ARM_HOME_SEEK;
    s_arm_homed = false;
    s_arm_pending.pending = true;
    s_arm_pending.target_steps = (int32_t)ARM_HOME_SEARCH_STEPS;
    s_arm_pending.subsys = SUBSYS_ARM;
    s_arm_pending.start_ms = to_ms_since_boot(get_absolute_time());
    s_arm_pending.timeout_ms = (uint32_t)ARM_HOME_TIMEOUT_MS;
    s_arm_pending.dev_idx = (uint8_t)STEPPER_DEV_ROT_ARM;

    printf("Arm home started: search=%u backoff=%u\n",
           (unsigned)ARM_HOME_SEARCH_STEPS,
           (unsigned)ARM_HOME_BACKOFF_STEPS);
    send_ack(seq);
}

/**
 * @brief Sensorlessly home the agitator to its mechanical endstop via stall
 *        detection, then back off AGITATOR_HOME_BACKOFF_STEPS.
 *
 * After homing the tick automatically starts the N-cycle knead sequence
 * (using s_agitator_n_cycles).  s_agitator_pos_steps is zeroed at the
 * endstop and updated through each subsequent stroke.
 *
 * @return true if homing was started, false if not wired / already running.
 */
static bool agitator_start_homing(void)
{
#ifdef STEPPER_DEV_AGITATOR
    if (!s_stepper_ready)
    {
        printf("Agitator homing: stepper not ready\n");
        return false;
    }
    if (DRV8434S_N_DEVICES <= (uint8_t)STEPPER_DEV_AGITATOR)
    {
        printf("Agitator homing: device %u not in chain (N=%u)\n",
               (unsigned)STEPPER_DEV_AGITATOR, (unsigned)DRV8434S_N_DEVICES);
        return false;
    }
    if (s_agitator_pending.pending)
    {
        printf("Agitator homing: already running\n");
        return false;
    }

    s_agitator_cycle_done  = false;
    s_agitator_cycles_done = 0u;
    s_agitator_phase       = AGIT_PHASE_HOMING;

    set_motion_torque_sample_div(STEPPER_DEFAULT_TORQUE_SAMPLE_DIV); /* sensitive stall detection for homing */
    if (!start_stepper_job((uint8_t)STEPPER_DEV_AGITATOR,
                           (int32_t)AGITATOR_HOME_SEARCH_STEPS,
                           (uint16_t)AGITATOR_HOME_TORQUE_LIMIT,
                           0u,
                           (uint32_t)AGITATOR_STEP_DELAY_US))
    {
        set_motion_torque_sample_div((uint8_t)STEPPER_DEFAULT_TORQUE_SAMPLE_DIV);
        printf("Agitator homing: start_stepper_job failed\n");
        s_agitator_phase = AGIT_PHASE_IDLE;
        return false;
    }

    s_agitator_pending.pending      = true;
    s_agitator_pending.target_steps = (int32_t)AGITATOR_HOME_SEARCH_STEPS;
    s_agitator_pending.subsys       = SUBSYS_AGITATOR;
    s_agitator_pending.start_ms     = to_ms_since_boot(get_absolute_time());
    s_agitator_pending.timeout_ms   = (uint32_t)AGITATOR_HOME_TIMEOUT_MS;
    s_agitator_pending.dev_idx      = (uint8_t)STEPPER_DEV_AGITATOR;

    printf("Agitator homing: started (search=%u torque=%u then %u cycles)\n",
           (unsigned)AGITATOR_HOME_SEARCH_STEPS,
           (unsigned)AGITATOR_HOME_TORQUE_LIMIT,
           (unsigned)s_agitator_n_cycles);
    return true;
#else
    printf("Agitator homing: STEPPER_DEV_AGITATOR not defined\n");
    return false;
#endif
}

/**
 * @brief Start N knead cycles (forward + reverse, N = s_agitator_n_cycles) on
 *        STEPPER_DEV_AGITATOR without homing first.  Caller must set
 *        s_agitator_n_cycles before calling.
 *
 * The agitator completion tick handles all phase transitions automatically.
 * s_agitator_cycle_done is set true when all strokes complete (or on fault).
 * MSG_MOTION_DONE(SUBSYS_AGITATOR) is sent at the end.
 *
 * @return true if the cycle was started, false if not wired / already running.
 */
static bool agitator_start_cycle(void)
{
#ifdef STEPPER_DEV_AGITATOR
    if (!s_stepper_ready)
    {
        printf("Agitator: stepper not ready\n");
        return false;
    }
    if (DRV8434S_N_DEVICES <= (uint8_t)STEPPER_DEV_AGITATOR)
    {
        printf("Agitator: device %u not in chain (N=%u)\n",
               (unsigned)STEPPER_DEV_AGITATOR, (unsigned)DRV8434S_N_DEVICES);
        return false;
    }
    if (s_agitator_pending.pending)
    {
        printf("Agitator: already running\n");
        return false;
    }

    s_agitator_cycle_done  = false;
    s_agitator_cycles_done = 0u;
    s_agitator_phase       = AGIT_PHASE_FORWARD;

    if (!start_stepper_job((uint8_t)STEPPER_DEV_AGITATOR,
                           (int32_t)AGITATOR_KNEAD_STEPS,
                           0u, 0u,
                           (uint32_t)AGITATOR_STEP_DELAY_US))
    {
        printf("Agitator: start_stepper_job (forward) failed\n");
        s_agitator_phase = AGIT_PHASE_IDLE;
        return false;
    }

    s_agitator_pending.pending      = true;
    s_agitator_pending.target_steps = (int32_t)AGITATOR_KNEAD_STEPS;
    s_agitator_pending.subsys       = SUBSYS_AGITATOR;
    s_agitator_pending.start_ms     = to_ms_since_boot(get_absolute_time());
    s_agitator_pending.timeout_ms   = (uint32_t)AGITATOR_MOTION_TIMEOUT_MS;
    s_agitator_pending.dev_idx      = (uint8_t)STEPPER_DEV_AGITATOR;

    printf("Agitator: starting %u×(fwd+rev) cycles (%u steps @ %u us/step)\n",
           (unsigned)s_agitator_n_cycles,
           (unsigned)AGITATOR_KNEAD_STEPS, (unsigned)AGITATOR_STEP_DELAY_US);
    return true;
#else
    printf("Agitator: STEPPER_DEV_AGITATOR not defined\n");
    return false;
#endif
}

/**
 * @brief MSG_AGITATE (0x4E) — trigger a standalone agitation cycle.
 *
 * Optional pl_agitate_t payload (len may be 0 for all defaults):
 *   flags    AGITATE_FLAG_DO_HOME → home only, do not enter knead cycle
 *   n_cycles 0 = use AGITATOR_N_CYCLES from board_pins.h; >0 overrides cycle count
 *
 * Pico sends MSG_MOTION_DONE(SUBSYS_AGITATOR, ...) when operation completes.
 * NACKs if stepper not ready, agitator not wired, or another cycle is running.
 */
static void handle_agitate(uint16_t seq, const uint8_t *payload, uint16_t len)
{
    if (s_agitator_pending.pending)
    {
        printf("Agitate: already in progress\n");
        send_nack(seq, NACK_UNKNOWN);
        return;
    }

    /* Parse optional payload */
    uint8_t flags    = 0u;
    uint8_t n_cycles = 0u;
    if (len >= 1u && payload)
        flags = payload[0];
    if (len >= 2u && payload)
        n_cycles = payload[1];

    s_agitator_n_cycles    = (n_cycles == 0u) ? (uint8_t)AGITATOR_N_CYCLES : n_cycles;
    s_agitator_cycles_done = 0u;

    bool started;
    if (flags & AGITATE_FLAG_DO_HOME)
    {
        s_agitator_home_only = true;
        started = agitator_start_homing();
    }
    else
    {
        s_agitator_home_only = false;
        started = agitator_start_cycle();
    }

    if (!started)
    {
        send_nack(seq, NACK_UNKNOWN);
        return;
    }

    send_ack(seq);
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

    if (arm_stepper_control_locked() || s_arm_rehome_required)
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
    case ARM_POS_HOME:
        /* Move back to released-home position (step 0) without re-homing.
         * Requires a prior MSG_ARM_HOME; already guarded by s_arm_rehome_required above. */
        target = 0;
        break;
    default:
        send_nack(seq, NACK_UNKNOWN);
        return;
    }

    printf("Arm move → %li steps\n", (long)target);
    if (start_stepper_motion((uint8_t)STEPPER_DEV_ROT_ARM, target, &s_arm_pending,
                             SUBSYS_ARM, (uint32_t)ARM_MOTION_TIMEOUT_MS, seq))
    {
        if (arm_press_retry_should_arm(target))
        {
            s_arm_press_retry.phase = ARM_PRESS_RETRY_PRESSING;
            s_arm_press_retry.retries_started = 0u;
            s_arm_press_retry.baseline_rpm = s_vacuum_last_rpm;
            s_arm_press_retry.phase_start_ms = to_ms_since_boot(get_absolute_time());
            printf("Arm press verify armed: baseline RPM=%u retries=%u backoff=%u delta=%u timeout=%u ms\n",
                   (unsigned)s_arm_press_retry.baseline_rpm,
                   (unsigned)ARM_PRESS_RETRY_MAX_RETRIES,
                   (unsigned)ARM_PRESS_RETRY_BACKOFF_STEPS,
                   (unsigned)ARM_PRESS_RETRY_RPM_DELTA,
                   (unsigned)ARM_PRESS_RETRY_VERIFY_TIMEOUT_MS);
        }
        else
        {
            reset_arm_press_retry();
        }
    }
    else
    {
        reset_arm_press_retry();
    }
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

    if (arm_stepper_control_locked())
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

#ifdef STEPPER_DEV_LIN_ARM
    printf("Rack move → %li steps\n", (long)target);
    (void)start_stepper_motion((uint8_t)STEPPER_DEV_LIN_ARM, target, &s_rack_pending,
                               SUBSYS_RACK, (uint32_t)RACK_MOTION_TIMEOUT_MS, seq);
#else
    /* No linear arm / rack device in the current chain configuration.
     * Define STEPPER_DEV_LIN_ARM in board_pins.h and bump DRV8434S_N_DEVICES
     * when the linear arm stepper is wired. */
    printf("Rack Move: STEPPER_DEV_LIN_ARM not defined (device not wired)\n");
    send_nack(seq, NACK_UNKNOWN);
    (void)target;
#endif
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

    if (arm_stepper_control_locked())
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
    case TURNTABLE_POS_INTAKE:
        target = (int32_t)TURNTABLE_STEPS_INTAKE;
        break;
    case TURNTABLE_POS_TRASH:
        target = (int32_t)TURNTABLE_STEPS_TRASH;
        break;
    case TURNTABLE_POS_EJECT:
        target = (int32_t)TURNTABLE_STEPS_EJECT;
        break;
    default:
        send_nack(seq, NACK_UNKNOWN);
        return;
    }

    printf("Turntable GOTO → %li steps\n", (long)target);
    (void)start_stepper_motion((uint8_t)STEPPER_DEV_TURNTABLE, target, &s_turntable_pending,
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
    if (arm_stepper_control_locked())
    {
        send_nack(seq, NACK_UNKNOWN);
        return;
    }

    /* Cancel any in-progress turntable motion. */
    if (s_motion.jobs[STEPPER_DEV_TURNTABLE].active)
    {
        drv8434s_motion_cancel(&s_motion, (uint8_t)STEPPER_DEV_TURNTABLE, NULL);
        s_turntable_pending.pending = false;
    }

    s_turntable_pos_steps = 0;
    s_turntable_homed = true;
    printf("Turntable HOME: position zeroed, homed=true\n");
    send_ack(seq);
}

/**
 * @brief MSG_HOTWIRE_SET (0x46) — enable or disable the hot-wire DRV8263.
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

    if (!s_drv8263_hotwire_ready)
    {
        send_nack(seq, NACK_UNKNOWN);
        return;
    }

    const pl_hotwire_set_t *p = (const pl_hotwire_set_t *)payload;

    if (p->enable)
    {
        /* DRV8263 independent H-bridge mode: IN1 drives the hot wire.
         * Current is set by an external Rsense resistor and regulated
         * internally — no PWM duty cycle tuning required; full on (4095).
         * IN2 (vacuum pump 2) is unaffected by this call. */
        (void)drv8263_set_in1(&s_drv8263_hotwire, (uint16_t)HOTWIRE_ENABLE_DUTY);
        printf("Hotwire ON (IN1, duty=%u)\n", (unsigned)HOTWIRE_ENABLE_DUTY);
    }
    else
    {
        /* De-assert IN1 only — vacuum pump 2 (IN2) continues unaffected. */
        (void)drv8263_set_in1(&s_drv8263_hotwire, 0u);
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
        s_vacuum_last_rpm = 0u;
        s_vacuum_rpm_valid = false;
        send_vacuum_status(VACUUM_OFF, 0u);
        printf("Vacuum OFF\n");
    }
    else
    {
        /* Reset sampling timestamps so the first sample fires promptly. */
        s_vacuum_last_sample_ms = to_ms_since_boot(get_absolute_time());
        s_vacuum_last_status_ms = s_vacuum_last_sample_ms;
        /* Reset pulse timing state atomically. */
        uint32_t saved = save_and_disable_interrupts();
        s_vacuum_pulse_count = 0u;
        s_vacuum_last_pulse_time_us = 0u;
        s_vacuum_last_period_us = 0u;
        s_vacuum_period_sum_us = 0u;
        s_vacuum_period_samples = 0u;
        restore_interrupts(saved);
        s_vacuum_last_rpm = 0u;
        s_vacuum_rpm_valid = false;
        printf("Vacuum ON\n");
    }

    send_ack(seq);
}

/**
 * @brief MSG_VACUUM2_SET (0x48) — enable or disable the second vacuum pump.
 *
 * DRV8263 independent H-bridge mode: IN2 (ctrl_b_pin) drives vacuum pump 2.
 * This is completely independent of IN1 (hot wire) — both can run simultaneously.
 * No mutual exclusion is required or applied.
 */
static void handle_vacuum2_set(uint16_t seq, const uint8_t *payload, uint16_t len)
{
    if (len < (uint16_t)sizeof(pl_vacuum2_set_t))
    {
        send_nack(seq, NACK_BAD_LEN);
        return;
    }

    if (!s_drv8263_hotwire_ready)
    {
        send_nack(seq, NACK_UNKNOWN);
        return;
    }

    const pl_vacuum2_set_t *pl = (const pl_vacuum2_set_t *)payload;

    if (pl->enable)
    {
        /* Drive IN2 independently at full duty — IN1 (hot wire) is unaffected. */
        (void)drv8263_set_in2(&s_drv8263_hotwire, 4095u);
        printf("Vacuum2 ON (DRV8263 IN2, independent)\n");
    }
    else
    {
        (void)drv8263_set_in2(&s_drv8263_hotwire, 0u);
        printf("Vacuum2 OFF\n");
    }

    send_ack(seq);
}

/**
 * @brief MSG_HOTWIRE_TRAVERSE (0x4B) — move the hot wire carriage stepper.
 *
 * Direction 0 = cut (traverse forward through spawn bag tip).
 * Direction 1 = return (retract to home position).
 * Uses DRV8434S STEPPER_DEV_HW_CARRIAGE (device 4 — not wired in current HW).
 * When device 4 is not in the chain, this handler logs and NACKs.
 */
static void handle_hotwire_traverse(uint16_t seq, const uint8_t *payload, uint16_t len)
{
    if (len < (uint16_t)sizeof(pl_hotwire_traverse_t))
    {
        send_nack(seq, NACK_BAD_LEN);
        return;
    }

#ifndef STEPPER_DEV_HW_CARRIAGE
    /* Hot wire carriage stepper not yet wired — NACK until device is added. */
    printf("Hotwire Traverse: STEPPER_DEV_HW_CARRIAGE not defined (device not wired)\n");
    send_nack(seq, NACK_UNKNOWN);
    (void)payload;
    return;
#else
    if (!s_stepper_ready || DRV8434S_N_DEVICES <= STEPPER_DEV_HW_CARRIAGE)
    {
        printf("Hotwire Traverse: stepper not ready or device not in chain\n");
        send_nack(seq, NACK_UNKNOWN);
        return;
    }

    if (arm_stepper_control_locked())
    {
        send_nack(seq, NACK_UNKNOWN);
        return;
    }

    const pl_hotwire_traverse_t *p = (const pl_hotwire_traverse_t *)payload;
    const uint32_t step_delay_us = (uint32_t)HOTWIRE_TRAVERSE_STEP_DELAY_US;
    const uint32_t us_to_ms_ceiling_divisor_offset = US_PER_MS - 1u;
    int32_t steps = (p->direction == 0u)
                        ? (int32_t)HOTWIRE_TRAVERSE_STEPS
                        : -(int32_t)HOTWIRE_TRAVERSE_STEPS;
    uint32_t hotwire_motion_us = (uint32_t)HOTWIRE_TRAVERSE_STEPS * step_delay_us;
    uint32_t hotwire_motion_ms = (hotwire_motion_us + us_to_ms_ceiling_divisor_offset) / US_PER_MS;
    /* Add guard to absorb scheduler jitter, motion-engine startup latency,
     * and minor tuning variance in step timing. */
    uint32_t timeout_ms = hotwire_motion_ms + HOTWIRE_TIMEOUT_GUARD_MS;

    if (s_motion.jobs[STEPPER_DEV_HW_CARRIAGE].active)
    {
        drv8434s_motion_cancel(&s_motion, (uint8_t)STEPPER_DEV_HW_CARRIAGE, NULL);
    }
    s_hotwire_pending.pending = false;

    if (!start_stepper_job((uint8_t)STEPPER_DEV_HW_CARRIAGE,
                           steps,
                           0u,
                           (uint16_t)DRV8434S_MOTION_TORQUE_BLANK_STEPS,
                           step_delay_us))
    {
        printf("Hotwire Traverse: failed to start motion\n");
        send_nack(seq, NACK_UNKNOWN);
        return;
    }

    s_hotwire_pending.pending = true;
    s_hotwire_pending.target_steps = s_hotwire_pos_steps + steps;
    s_hotwire_pending.subsys = SUBSYS_HOTWIRE;
    s_hotwire_pending.start_ms = to_ms_since_boot(get_absolute_time());
    s_hotwire_pending.timeout_ms = timeout_ms;
    s_hotwire_pending.dev_idx = (uint8_t)STEPPER_DEV_HW_CARRIAGE;

    printf("Hotwire Traverse: dir=%u steps=%li\n",
           (unsigned)p->direction, (long)steps);
    send_ack(seq);
#endif
}

/**
 * @brief MSG_INDEXER_MOVE (0x4C) — move bag depth/eject rack to named position.
 *
 * INDEXER_POS_OPEN   — retracted (bag can slide in).
 * INDEXER_POS_CENTER — extended to bag-centering position.
 * INDEXER_POS_EJECT  — fully extended to push bag out.
 * Uses DRV8434S STEPPER_DEV_INDEXER (device 5 — not wired in current HW).
 */
static void handle_indexer_move(uint16_t seq, const uint8_t *payload, uint16_t len)
{
    if (len < (uint16_t)sizeof(pl_indexer_move_t))
    {
        send_nack(seq, NACK_BAD_LEN);
        return;
    }

#ifndef STEPPER_DEV_INDEXER
    printf("Indexer Move: STEPPER_DEV_INDEXER not defined (device not wired)\n");
    send_nack(seq, NACK_UNKNOWN);
    (void)payload;
    return;
#else
    if (!s_stepper_ready || DRV8434S_N_DEVICES <= STEPPER_DEV_INDEXER)
    {
        printf("Indexer Move: stepper not ready or device not in chain\n");
        send_nack(seq, NACK_UNKNOWN);
        return;
    }

    if (arm_stepper_control_locked())
    {
        send_nack(seq, NACK_UNKNOWN);
        return;
    }

    const pl_indexer_move_t *p = (const pl_indexer_move_t *)payload;
    int32_t target_steps;

    switch ((indexer_pos_t)p->position)
    {
    case INDEXER_POS_OPEN:
        target_steps = 0;
        break;
    case INDEXER_POS_CENTER:
        target_steps = (int32_t)INDEXER_STEPS_CENTER;
        break;
    case INDEXER_POS_EJECT:
        target_steps = (int32_t)INDEXER_STEPS_EJECT;
        break;
    default:
        printf("Indexer Move: unknown position %u\n", (unsigned)p->position);
        send_nack(seq, NACK_UNKNOWN);
        return;
    }

    int32_t delta = target_steps - s_indexer_pos_steps;

    if (delta == 0)
    {
        /* Already at target — send immediate MOTION_DONE. */
        printf("Indexer Move: already at position %u (no-op)\n", (unsigned)p->position);
        send_ack(seq);
        send_motion_done(SUBSYS_INDEXER, MOTION_OK, s_indexer_pos_steps);
        return;
    }

    if (!start_stepper_job((uint8_t)STEPPER_DEV_INDEXER,
                           delta,
                           0u,
                           (uint16_t)DRV8434S_MOTION_TORQUE_BLANK_STEPS,
                           (uint32_t)STEPPER_DEFAULT_STEP_DELAY_US))
    {
        printf("Indexer Move: failed to start motion\n");
        send_nack(seq, NACK_UNKNOWN);
        return;
    }

    s_indexer_pos_steps = target_steps;
    printf("Indexer Move: pos=%u delta=%li %s\n",
           (unsigned)p->position, (long)delta,
           (delta < 0) ? "REV" : "FWD");
    send_ack(seq);
    send_motion_done(SUBSYS_INDEXER, MOTION_OK, s_indexer_pos_steps);
#endif
}

/**
 * @brief MSG_DISPENSE_SPAWN (0x49) - run the improved closed-loop dosing procedure.
 *
 * Supports two finish strategies (Finish A: close-early + top-off; Finish B: low-flow
 * taper) selectable via the flags byte in pl_innoculate_bag_t.  Also supports
 * optional homing to the closed endpoint before priming.
 *
 * Payload: pl_innoculate_bag_t (8 bytes; accepts 7-byte legacy payloads with flags=0).
 */
static void handle_dispense_spawn(uint16_t seq, const uint8_t *payload, uint16_t len)
{
    if (len < (uint16_t)PL_INNOCULATE_BAG_MIN_LEN)
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

    /* Read optional flags byte (backward-compat: absent = 0) */
    uint8_t flags = (len >= (uint16_t)sizeof(pl_innoculate_bag_t)) ? pl->flags : 0u;
    uint8_t finish_mode = (flags & SPAWN_FLAG_FINISH_MODE_B) ? (uint8_t)SPAWN_FINISH_MODE_B
                                                              : (uint8_t)SPAWN_FINISH_MODE_A;
    bool do_home = (bool)(flags & SPAWN_FLAG_DO_HOME);

    printf("Spawn: finish_mode=%u do_home=%u flags=0x%02x\n",
           (unsigned)finish_mode, (unsigned)do_home, (unsigned)flags);

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
    s_spawn.active      = true;
    s_spawn.agitating   = false;
    s_spawn.retry       = 0u;
    s_spawn.max_retries = SPAWN_MAX_RETRIES;
    s_spawn.bag_number  = pl->bag_number;
    s_spawn.finish_mode = finish_mode;
    s_spawn.do_home     = do_home;
    s_spawn.start_mass_ug  = (uint32_t)mass.ug;
    /* innoc_percent is x10 percent (e.g., 250 = 25.0%) */
    uint64_t target_ug = ((uint64_t)s_spawn.start_mass_ug * (uint64_t)pl->innoc_percent) / 1000ULL;
    s_spawn.target_ug          = (uint32_t)target_ug;
    s_spawn.dispensed_ug       = 0u;
    s_spawn.window_start_ug    = 0u;
    s_spawn.last_tick_ug       = 0u;
    s_spawn.spawn_remaining_ug = ((uint32_t)pl->spawn_mass) * 1000000UL;
    s_spawn.last_flow_check    = get_absolute_time();
    s_spawn.nudging            = false;
    s_spawn.ema_seeded         = false;
    s_spawn.ema_flow_ug        = 0u;
    s_spawn.consecutive_open_nudges = 0u;
    s_spawn.last_nudge_dir     = 0;
    s_spawn.reverse_holdoff_until = get_absolute_time();
    s_spawn.topoff_pulses      = 0u;
    s_spawn.topoff_settling    = false;

    double m;
    mass_get_value(&mass, &m);
    printf("Spawn: initial mass %.3fg  target=%luug  finish=%u  home=%u\n",
           m, (unsigned long)s_spawn.target_ug, (unsigned)finish_mode, (unsigned)do_home);

    if (do_home)
    {
        /* Home: drive flaps to closed endpoint, then wait in SPAWN_SM_HOMING. */
        if (s_drv8263.monitoring_enabled)
        {
            drv8263_stop_current_monitoring(&s_drv8263);
        }
        /* flap_close_internal() sets up the DRV8263, starts the flap SM, and  *
         * sets s_flap_state = FLAP_SM_CLOSING so flap_sm_tick() can detect    *
         * the endpoint.  spawn callback polls s_flap_state for completion.    */
        (void)flap_close_internal();
        s_spawn.close_start_time = get_absolute_time();
        s_spawn.startup_opening  = false;
        spawn_set_state(SPAWN_SM_HOMING);
    }
    else
    {
        /* No homing — drive open immediately and go to PRIME */
        s_spawn.startup_opening = true;

        if (s_drv8263.monitoring_enabled)
        {
            drv8263_stop_current_monitoring(&s_drv8263);
        }
        s_drv8263.config.current_low_threshold     = (uint16_t)FLAP_OPEN_CURRENT_DROP_TH;
        s_drv8263.config.current_high_threshold    = 4095u;
        s_drv8263.config.current_check_interval_ms = (uint32_t)FLAP_MONITOR_INTERVAL_MS;
        spawn_set_flap_pwm(true, (uint16_t)SPAWN_OPEN_PWM);
        (void)drv8263_start_current_monitoring(&s_drv8263);

        if (s_drv8263_flap2_ready)
        {
            if (s_drv8263_flap2.monitoring_enabled)
            {
                drv8263_stop_current_monitoring(&s_drv8263_flap2);
            }
            s_drv8263_flap2.config.current_low_threshold     = (uint16_t)FLAP_OPEN_CURRENT_DROP_TH;
            s_drv8263_flap2.config.current_high_threshold    = 4095u;
            s_drv8263_flap2.config.current_check_interval_ms = (uint32_t)FLAP_MONITOR_INTERVAL_MS;
            (void)drv8263_set_motor_control(&s_drv8263_flap2, DRV8263_MOTOR_FORWARD,
                                            (uint16_t)SPAWN_OPEN_PWM);
            (void)drv8263_start_current_monitoring(&s_drv8263_flap2);
        }

        spawn_set_state(SPAWN_SM_PRIME);
    }

    if (!add_repeating_timer_ms((int32_t)SPAWN_TIMER_PERIOD_MS, dispense_spawn_callback,
                                NULL, &s_spawn.timer))
    {
        spawn_close_flaps();
        s_spawn.active = false;
        send_nack(seq, NACK_UNKNOWN);
        return;
    }

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

    case MSG_MOTOR_DRV8263_START_MON:
        printf("DRV8263 Start Mon\n");
        handle_drv8263_start(hdr.seq, payload, hdr.len);
        break;

    case MSG_MOTOR_DRV8263_STOP_MON:
        printf("DRV8263 Stop Mon\n");
        handle_drv8263_stop(hdr.seq);
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

    case MSG_ARM_HOME:
        printf("Arm Home\n");
        handle_arm_home(hdr.seq);
        break;

    case MSG_AGITATE:
        printf("Agitate\n");
        handle_agitate(hdr.seq, payload, hdr.len);
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

    case MSG_HOTWIRE_TRAVERSE:
        printf("Hotwire Traverse\n");
        handle_hotwire_traverse(hdr.seq, payload, hdr.len);
        break;

    case MSG_INDEXER_MOVE:
        printf("Indexer Move\n");
        handle_indexer_move(hdr.seq, payload, hdr.len);
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
            spawn_stop_flaps();
            spawn_fast_close();  /* fast close even on abort for safety */
            s_spawn.active = false;
            s_spawn.agitating = false;
            spawn_set_state(SPAWN_SM_ABORTED);
            printf("Spawn ABORT: stopped at state=%u disp=%luug target=%luug\n",
                   (unsigned)s_spawn.prev_state,
                   (unsigned long)s_spawn.dispensed_ug,
                   (unsigned long)s_spawn.target_ug);
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
 * @brief Initialise the second flap DRV8263 instance.
 *
 * Uses FLAP2_CTRL_A_PIN / FLAP2_CTRL_B_PIN / FLAP2_ADC_SENSE_PIN /
 * FLAP2_ADC_CHANNEL from board_pins.h.  Shares all threshold constants
 * with the primary flap instance.
 */
static void init_drv8263_flap2(void)
{
    s_drv8263_flap2_ready = false;

    drv8263_config_t cfg = {
        .user_ctx = NULL,
        .ctrl_a_pin = (uint8_t)FLAP2_CTRL_A_PIN,
        .ctrl_b_pin = (uint8_t)FLAP2_CTRL_B_PIN,
        .sense_pin = (uint8_t)FLAP2_ADC_SENSE_PIN,
        .sense_adc_channel = (uint8_t)FLAP2_ADC_CHANNEL,
        .current_high_threshold = (uint16_t)DRV8263_DEFAULT_HIGH_TH,
        .current_low_threshold = (uint16_t)DRV8263_DEFAULT_LOW_TH,
        .pwm_frequency_hz = (uint32_t)DRV8263_DEFAULT_PWM_HZ,
        .current_check_interval_ms = (uint32_t)FLAP_MONITOR_INTERVAL_MS,
        .startup_blanking_ms = (uint32_t)DRV8263_DEFAULT_STARTUP_BLANKING_MS,
        .current_cb = NULL,
        .delay_ms = NULL,
    };

    if (!drv8263_init(&s_drv8263_flap2, &cfg))
    {
        printf("uart_server: DRV8263 flap2 init failed\n");
        return;
    }

    s_drv8263_flap2_ready = true;
    printf("uart_server: DRV8263 flap2 ready (A=%d B=%d ADC_CH=%d)\n",
           FLAP2_CTRL_A_PIN, FLAP2_CTRL_B_PIN, FLAP2_ADC_CHANNEL);
}

static void init_drv8263(void)
{
    s_drv8263_ready = false;

    if (DRV8263_CTRL_A_GPIO < 0 || DRV8263_CTRL_B_GPIO < 0 || DRV8263_SENSE_GPIO < 0 ||
        DRV8263_SENSE_ADC_CH < 0 || DRV8263_SENSE_ADC_CH > 3)
    {
        printf("uart_server: DRV8263 pins/ch not set in board_pins.h; motor RPC disabled\n");
        return;
    }

    drv8263_config_t cfg = {
        .user_ctx = NULL,
        .ctrl_a_pin = (uint8_t)DRV8263_CTRL_A_GPIO,
        .ctrl_b_pin = (uint8_t)DRV8263_CTRL_B_GPIO,
        .sense_pin = (uint8_t)DRV8263_SENSE_GPIO,
        .sense_adc_channel = (uint8_t)DRV8263_SENSE_ADC_CH,
        .current_high_threshold = DRV8263_DEFAULT_HIGH_TH,
        .current_low_threshold = DRV8263_DEFAULT_LOW_TH,
        .pwm_frequency_hz = DRV8263_DEFAULT_PWM_HZ,
        .current_check_interval_ms = DRV8263_DEFAULT_CHECK_INTERVAL_MS,
        .startup_blanking_ms = DRV8263_DEFAULT_STARTUP_BLANKING_MS,
        .current_cb = NULL,
        .delay_ms = NULL,
    };

    if (!drv8263_init(&s_drv8263, &cfg))
    {
        printf("uart_server: DRV8263 init failed\n");
        return;
    }

    s_drv8263_ready = true;
    printf("uart_server: DRV8263 flap1 ready (A=%d B=%d ADC_CH=%d)\n",
           DRV8263_CTRL_A_GPIO, DRV8263_CTRL_B_GPIO, DRV8263_SENSE_ADC_CH);
}

/**
 * @brief Initialise the hot-wire DRV8263 instance (secondary H-bridge).
 *
 * Hot wire is unidirectional (forward only, constant current duty cycle).
 * The current monitoring thresholds are set wide — we do not use auto-stop
 * for the hot wire; ON/OFF is controlled entirely by MSG_HOTWIRE_SET.
 */
static void init_drv8263_hotwire(void)
{
    s_drv8263_hotwire_ready = false;

    drv8263_config_t cfg = {
        .user_ctx = NULL,
        .ctrl_a_pin = (uint8_t)HOTWIRE_PIN_IN1,
        .ctrl_b_pin = (uint8_t)HOTWIRE_PIN_IN2,
        .sense_pin = (uint8_t)HOTWIRE_ADC_SENSE_PIN,
        .sense_adc_channel = (uint8_t)HOTWIRE_ADC_CHANNEL,
        .current_high_threshold = 4095u, /* never auto-stop */
        .current_low_threshold = 0u,     /* never auto-stop */
        .pwm_frequency_hz = 20000u,
        .current_check_interval_ms = (uint32_t)HOTWIRE_MONITOR_INTERVAL_MS,
        .startup_blanking_ms = (uint32_t)DRV8263_DEFAULT_STARTUP_BLANKING_MS,
        .current_cb = NULL,
        .delay_ms = NULL,
    };

    if (!drv8263_init(&s_drv8263_hotwire, &cfg))
    {
        printf("uart_server: DRV8263 hotwire init failed\n");
        return;
    }

    s_drv8263_hotwire_ready = true;
    printf("uart_server: DRV8263 hotwire ready (IN1=%d IN2=%d ADC_CH=%d)\n",
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

    /* Ensure the chain is not in a faulted state at startup. */
    (void)drv8434s_chain_global_clear_faults(&s_stepper_chain);

    /* Configure each device for Smart-Tune Ripple mode + stall detection. */
    for (uint8_t k = 0; k < s_stepper_chain.cfg.n_devices; ++k)
    {
        /* Configure fault reporting (so stall detection is allowed by default). */
        (void)drv8434s_chain_set_fault_config(&s_stepper_chain, k,
                                             DRV8434S_FAULT_CFG_STL_REP,
                                             NULL);

        /* Smart-tune ripple decay is required for TRQ_COUNT to update. */
        (void)drv8434s_chain_modify_reg(&s_stepper_chain, k,
                                        DRV8434S_REG_CTRL2,
                                        DRV8434S_CTRL2_DECAY_MASK,
                                        (uint8_t)DRV8434S_DECAY_SMART_TUNE_RIPPLE,
                                        NULL);

        /* Enable stall detection (EN_STL). */
        (void)drv8434s_chain_modify_reg(&s_stepper_chain, k,
                                        DRV8434S_REG_CTRL5,
                                        DRV8434S_CTRL5_EN_STL,
                                        DRV8434S_CTRL5_EN_STL,
                                        NULL);

        /* Set stall threshold to minimum (1) so TRQ_COUNT updates.
         * When set to 0 the device may not update torque count at all.
         * A real stall will still produce TRQ_COUNT=0 and can be detected
         * by our soft limit logic. */
        (void)drv8434s_chain_write_reg(&s_stepper_chain, k,
                                      DRV8434S_REG_CTRL6, 1u, NULL);
        (void)drv8434s_chain_modify_reg(&s_stepper_chain, k,
                                        DRV8434S_REG_CTRL7,
                                        DRV8434S_CTRL7_STALL_TH_HI_MASK,
                                        0u,
                                        NULL);

        /* Enable torque-count scaling (optional but matches datasheet note).
         * This improves resolution for low-torque behavior. */
        (void)drv8434s_chain_modify_reg(&s_stepper_chain, k,
                                        DRV8434S_REG_CTRL7,
                                        DRV8434S_CTRL7_TRQ_SCALE,
                                        DRV8434S_CTRL7_TRQ_SCALE,
                                        NULL);
    }

    /* Initialise the non-blocking motion engine.
     * Default torque sampling divisor comes from board_pins.h. */
    if (!drv8434s_motion_init(&s_motion, &s_stepper_chain,
                              stepper_motion_done, NULL,
                              (uint8_t)STEPPER_DEFAULT_TORQUE_SAMPLE_DIV))
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

    /* Per-device SPI health probe: read FAULT register (0x00) from each
     * device.  A read failure indicates a persistent bus or wiring problem.
     * Log the result per device so failures are visible at boot. */
    for (uint8_t k = 0u; k < (uint8_t)DRV8434S_N_DEVICES; ++k)
    {
        uint8_t fault_val = 0u;
        if (drv8434s_chain_read_reg(&s_stepper_chain, k,
                                    DRV8434S_REG_FAULT, &fault_val, NULL))
        {
            printf("uart_server: DRV8434S dev %u SPI probe OK, FAULT=0x%02X%s\n",
                   (unsigned)k, (unsigned)fault_val,
                   (fault_val != 0u) ? " (FAULT ACTIVE)" : "");
        }
        else
        {
            printf("uart_server: DRV8434S dev %u SPI probe FAILED — check wiring\n",
                   (unsigned)k);
        }
    }
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
    s_vacuum_last_pulse_time_us = 0u;
    s_vacuum_last_period_us = 0u;
    s_vacuum_period_sum_us = 0u;
    s_vacuum_period_samples = 0u;
    s_vacuum_last_sample_ms = 0u;
    s_vacuum_last_status_ms = 0u;
    s_vacuum_status = VACUUM_OFF;
    s_vacuum_last_rpm = 0u;
    s_vacuum_rpm_valid = false;

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
    memset(&s_hotwire_pending, 0, sizeof(s_hotwire_pending));
    memset(&s_agitator_pending, 0, sizeof(s_agitator_pending));
    memset(&s_turntable_pending, 0, sizeof(s_turntable_pending));

    s_flap_state = FLAP_SM_IDLE;
    s_arm_pos_steps = 0;
    s_arm_homing_phase = ARM_HOME_IDLE;
    s_arm_rehome_required = false;
    s_arm_homed = false;
    reset_arm_press_retry();
    s_rack_pos_steps = 0;
    s_turntable_pos_steps = 0;
    s_hotwire_pos_steps = 0;
    /* Turntable requires explicit homing before any GOTO command is valid.
     * MSG_TURNTABLE_HOME must be sent first; GOTO commands in UNCALIBRATED
     * state are NACKed to prevent uncontrolled turntable movement. */
    s_turntable_homed = false;

    s_uart = (PICO_UART_ID == 0) ? uart0 : uart1;
    s_uart_ready = false;
    s_step_timer_active = false;
    s_step_timer_period_us = 0u;

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
    init_drv8263();
    init_drv8263_flap2();
    init_hx711();
    init_drv8434s_chain();

    /* New subsystems */
    init_drv8263_hotwire();
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
    arm_press_retry_tick();
    stepper_spi_watchdog_tick();

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
