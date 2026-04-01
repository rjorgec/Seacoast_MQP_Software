#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PROTO_VERSION 1u
#define PROTO_MAX_PAYLOAD 128u
#define PROTO_DELIM 0x00u

/* Shared default for DRV8434S soft torque-limit threshold.
 * Both ESP and Pico firmware should source their fallback from this symbol
 * so the UART payload default cannot drift between projects.
 * 100u is calibrated to avoid false stall trips seen with the previous
 * 300u default during routine moves. */
#define PROTO_STEPPER_SOFT_TORQUE_LIMIT_DEFAULT 100u

/*
 * Wire format endianness is little-endian for all multi-byte fields.
 * ESP32-C6 and RP2040 are both little-endian, so native packed structs
 * are used consistently on both sides.
 */

typedef enum {
  MSG_PING = 0x01,

  MSG_MOTOR_DRV8263_START_MON = 0x10,
  MSG_MOTOR_DRV8263_STOP_MON = 0x11,

  MSG_MOTOR_STEPPER_ENABLE = 0x20,
  MSG_MOTOR_STEPPER_STEPJOB = 0x21,

  MSG_HX711_TARE = 0x28,
  MSG_HX711_MEASURE = 0x29,

  MSG_CTRL_TARE = 0x30,
  MSG_CTRL_START = 0x31,
  MSG_CTRL_STOP = 0x32,
  MSG_CTRL_PAUSE = 0x33,

  /* ---- State-based command messages (0x40–0x5F) ---- */
  MSG_FLAPS_OPEN =
      0x40, /* ESP→Pico: open both flaps to open-circuit endpoint */
  MSG_FLAPS_CLOSE = 0x41,    /* ESP→Pico: close flaps up to torque threshold */
  MSG_ARM_MOVE = 0x42,       /* ESP→Pico: move arm stepper to named position */
  MSG_RACK_MOVE = 0x43,      /* ESP→Pico: move rack stepper to named position */
  MSG_TURNTABLE_GOTO = 0x44, /* ESP→Pico: move turntable to named position */
  MSG_TURNTABLE_HOME =
      0x45,               /* ESP→Pico: home turntable (zero position counter) */
  MSG_HOTWIRE_SET = 0x46, /* ESP→Pico: enable/disable hot wire PWM */
  MSG_VACUUM_SET = 0x47,  /* ESP→Pico: turn vacuum pump on/off */
  /** MSG_VACUUM2_SET — drives the independent half-bridge output IN2 of
   *  the hotwire/vacuum2 DRV8263.  NOT mutually exclusive with the hot
   *  wire (IN1); both outputs can be active simultaneously. */
  MSG_VACUUM2_SET = 0x48,    /* ESP→Pico: turn second vacuum pump on/off */
  MSG_DISPENSE_SPAWN = 0x49, /* ESP-Pico start closed-loop dosing*/
  MSG_SPAWN_STATUS = 0x4A,   /* Pico→ESP: spawn dosing status (unsolicited) */
  MSG_HOTWIRE_TRAVERSE =
      0x4B,                /* ESP→Pico: traverse hot wire carriage stepper */
  MSG_INDEXER_MOVE = 0x4C, /* ESP→Pico: move bag depth/eject rack (indexer) */
  MSG_ARM_HOME = 0x4D,     /* ESP→Pico: sensorless home for the rotary arm */
  MSG_AGITATE =
      0x4E, /* ESP→Pico: agitation cycle (optional pl_agitate_t payload) */
  /* ---- Unsolicited status messages (0x60–0x6F) ---- */
  MSG_MOTION_DONE = 0x60,   /* Pico→ESP: motion/action complete notification */
  MSG_VACUUM_STATUS = 0x61, /* Pico→ESP: vacuum pump RPM/blocked status */

  MSG_ACK = 0x80,
  MSG_NACK = 0x81,
} msg_type_t;

typedef enum {
  NACK_BAD_FRAME = 1,
  NACK_BAD_CRC = 2,
  NACK_BAD_LEN = 3,
  NACK_BAD_VERSION = 4,
  NACK_UNKNOWN = 5,
} nack_code_t;

typedef struct __attribute__((packed)) {
  uint8_t version;
  uint8_t type;
  uint16_t seq;
  uint16_t len;
} proto_hdr_t;

// typedef struct __attribute__((packed)) {
//     uint16_t speed;
//     uint16_t low_th;
//     uint16_t high_th;
//     uint32_t interval_ms;
// } pl_drv8263_start_mon_t;

typedef struct __attribute__((packed)) {
  uint8_t dir;    // 0=fwd, 1=rev
  uint8_t _rsvd;  // keep 16-bit alignment
  uint16_t speed; // 0..4095
  uint16_t low_th;
  uint16_t high_th;
  uint32_t interval_ms;
} pl_drv8263_start_mon_t;

typedef struct __attribute__((packed)) {
  uint32_t interval_us;
} pl_hx711_measure_t;

typedef struct __attribute__((packed)) {
  int32_t mass_ug;  // Mass in micrograms (internal representation)
  uint8_t unit;     // mass_unit_t enum value
  uint8_t _rsvd[3]; // Padding for alignment
} pl_hx711_mass_t;

typedef struct __attribute__((packed)) {
  uint8_t enable;
} pl_stepper_enable_t;

typedef struct __attribute__((packed)) {
  uint8_t dir;
  uint32_t steps;
  uint32_t step_delay_us;
  uint16_t torque_limit; /* soft torque limit (0 = disabled) */
} pl_stepper_stepjob_t;

typedef struct __attribute__((packed)) {
  uint8_t code;
} pl_nack_t;

/**
 * Finish-mode selector for MSG_DISPENSE_SPAWN.
 * Carried in pl_innoculate_bag_t::flags bit 0.
 */
typedef enum {
  SPAWN_FINISH_MODE_A = 0, /* close-early + top-off (anti-overshoot) */
  SPAWN_FINISH_MODE_B = 1, /* low-flow taper near target              */
} spawn_finish_mode_t;

/** Bit definitions for pl_innoculate_bag_t::flags */
#define SPAWN_FLAG_FINISH_MODE_B                                               \
  (1u << 0) /* set = Finish B, clear = Finish A */
#define SPAWN_FLAG_DO_HOME                                                     \
  (1u << 1) /* set = home flaps to closed before dosing */

typedef struct __attribute__((packed)) {
  uint16_t bag_mass;      // mass of bag being innoculated
  uint16_t spawn_mass;    // mass of spawn remaining
  uint16_t innoc_percent; // spawn percentage of bag weight (x10)
  uint8_t bag_number; // how many bags have been innoculated from the same spawn
  uint8_t flags; // SPAWN_FLAG_* bitmask (finish mode, homing); 0 = defaults
} pl_innoculate_bag_t;

/** Minimum payload length for pl_innoculate_bag_t without flags (legacy). */
#define PL_INNOCULATE_BAG_MIN_LEN 7u

typedef enum {
  SPAWN_STATUS_RUNNING = 0,
  SPAWN_STATUS_DONE = 1,
  SPAWN_STATUS_STALLED = 2,
  SPAWN_STATUS_AGITATING = 3,
  SPAWN_STATUS_BAG_EMPTY = 4,
  SPAWN_STATUS_ERROR = 5,
  SPAWN_STATUS_FLOW_FAILURE =
      6, /* flaps opened fully with no flow — bag likely empty or jammed */
  SPAWN_STATUS_ABORTED = 7, /* dose cancelled by MSG_CTRL_STOP */
} spawn_status_code_t;

typedef struct __attribute__((packed)) {
  uint8_t status;      /* spawn_status_code_t */
  uint8_t retries;     /* current retry count */
  uint16_t bag_number; /* copy of bag number from request */
  uint32_t target_ug;  /* target dispense mass in micrograms */
  uint32_t disp_ug;    /* dispensed mass in micrograms */
  uint32_t remain_ug;  /* estimated spawn remaining (if provided) */
} pl_spawn_status_t;

uint16_t proto_crc16_ccitt(const uint8_t *data, uint32_t len);

/* ------------------------------------------------------------------ */
/*  Supporting enumerations for state-based commands                   */
/* ------------------------------------------------------------------ */

/** Identifies which subsystem a MSG_MOTION_DONE refers to */
typedef enum {
  SUBSYS_FLAPS = 0,
  SUBSYS_ARM = 1,
  SUBSYS_RACK = 2,
  SUBSYS_TURNTABLE = 3,
  SUBSYS_HOTWIRE = 4,
  SUBSYS_VACUUM = 5,
  SUBSYS_INDEXER = 6,  /* Bag depth/eject rack (MSG_INDEXER_MOVE) */
  SUBSYS_AGITATOR = 7, /* Agitator eccentric arm (MSG_AGITATE) */
} subsystem_id_t;

/** Result code carried in MSG_MOTION_DONE */
typedef enum {
  MOTION_OK = 0,        /* reached target position / state */
  MOTION_STALLED = 1,   /* stall detected before target */
  MOTION_TIMEOUT = 2,   /* motion did not complete within deadline */
  MOTION_FAULT = 3,     /* driver fault (nFAULT asserted) */
  MOTION_SPI_FAULT = 4, /* SPI communication failure (not a physical stall) */
} motion_result_t;

/** Named positions for the arm stepper (DRV8434S device 0) */
typedef enum {
  ARM_POS_PRESS = 0, /* press against attachment point */
  ARM_POS_1 = 1,     /* absolute position 1 */
  ARM_POS_2 = 2,     /* absolute position 2 */
  ARM_POS_HOME =
      3, /* return to released-home position (step 0, set by MSG_ARM_HOME) */
} arm_pos_t;

/** Named positions for the rack stepper (DRV8434S device 1) */
typedef enum {
  RACK_POS_HOME = 0,   /* drive to physical end-stop and zero */
  RACK_POS_EXTEND = 1, /* move to extend position */
  RACK_POS_PRESS = 2,  /* move to press-into-arm position */
} rack_pos_t;

/** Named positions for the turntable stepper (DRV8434S device 2).
 *  TURNTABLE_POS_INTAKE (0) is the physical hard endstop; MSG_TURNTABLE_HOME
 *  drives to this position and zeros the step counter. */
typedef enum {
  TURNTABLE_POS_INTAKE = 0, /* bag-intake / home position (hard endstop) */
  TURNTABLE_POS_TRASH = 1,  /* waste-disposal position */
  TURNTABLE_POS_EJECT = 2,  /* bag-eject / output position */
} turntable_pos_t;

/** Vacuum pump status codes (carried in MSG_VACUUM_STATUS) */
typedef enum {
  VACUUM_OK = 0,
  VACUUM_BLOCKED = 1,
  VACUUM_OFF = 2,
} vacuum_status_code_t;

/** Bit flags for pl_agitate_t::flags */
#define AGITATE_FLAG_DO_HOME                                                   \
  (1u << 0) /**< home agitator before cycling (stall-detect) */

/**
 * MSG_AGITATE (0x4E) optional payload.
 * If len == 0 the Pico uses all defaults from board_pins.h.
 * Fields are read only up to the received length, so older senders remain
 * compatible.
 */
typedef struct __attribute__((packed)) {
  uint8_t flags; /**< AGITATE_FLAG_* bitmask; 0 = defaults */
  uint8_t
      n_cycles; /**< 0 = use AGITATOR_N_CYCLES from firmware; >0 overrides */
} pl_agitate_t;

/* ------------------------------------------------------------------ */
/*  Payload structs for state-based messages                           */
/*  All structs are packed; max total payload <= 128 bytes             */
/* ------------------------------------------------------------------ */

/** MSG_FLAPS_OPEN / MSG_FLAPS_CLOSE -- no variable payload needed;
 *  thresholds come from board_pins.h constants on the Pico side.
 *  Use proto_hdr_t with len=0 for both. */

/** MSG_ARM_MOVE payload (1 byte) */
typedef struct __attribute__((packed)) {
  uint8_t position; /**< arm_pos_t cast to uint8_t */
} pl_arm_move_t;

/** MSG_RACK_MOVE payload (1 byte) */
typedef struct __attribute__((packed)) {
  uint8_t position; /**< rack_pos_t cast to uint8_t */
} pl_rack_move_t;

/** MSG_TURNTABLE_GOTO payload (1 byte) */
typedef struct __attribute__((packed)) {
  uint8_t position; /**< turntable_pos_t cast to uint8_t */
} pl_turntable_goto_t;

/** MSG_HOTWIRE_SET payload (1 byte) */
typedef struct __attribute__((packed)) {
  uint8_t enable; /**< 1 = on, 0 = off */
} pl_hotwire_set_t;

/** MSG_VACUUM_SET payload (1 byte) */
typedef struct __attribute__((packed)) {
  uint8_t enable; /**< 1 = on, 0 = off */
} pl_vacuum_set_t;

/** MSG_VACUUM2_SET payload (1 byte) — DRV8263 independent IN2 half-bridge */
typedef struct __attribute__((packed)) {
  uint8_t enable; /**< 1 = on, 0 = off */
} pl_vacuum2_set_t;

/** Named positions for the indexer (bag depth/eject rack, STEPPER_DEV_INDEXER)
 */
typedef enum __attribute__((packed)) {
  INDEXER_POS_OPEN = 0,   /**< Retracted — bag can slide in */
  INDEXER_POS_CENTER = 1, /**< Extended to bag-centering position */
  INDEXER_POS_EJECT = 2,  /**< Fully extended — push bag out */
} indexer_pos_t;

/** MSG_HOTWIRE_TRAVERSE (0x4B) payload (1 byte) */
typedef struct __attribute__((packed)) {
  uint8_t direction; /**< 0 = cut (forward traverse), 1 = return (retrace/home; Pico zeroes position on success) */
} pl_hotwire_traverse_t;

/** MSG_INDEXER_MOVE (0x4C) payload (1 byte) */
typedef struct __attribute__((packed)) {
  uint8_t position; /**< indexer_pos_t cast to uint8_t */
} pl_indexer_move_t;

/** MSG_MOTION_DONE payload (8 bytes) -- unsolicited Pico->ESP */
typedef struct __attribute__((packed)) {
  uint8_t subsystem;  /**< subsystem_id_t */
  uint8_t result;     /**< motion_result_t */
  uint8_t _rsvd[2];   /**< reserved, set to 0 */
  int32_t steps_done; /**< signed step count actually executed */
} pl_motion_done_t;

/** MSG_VACUUM_STATUS payload (4 bytes) -- unsolicited Pico->ESP */
typedef struct __attribute__((packed)) {
  uint8_t status; /**< vacuum_status_code_t */
  uint8_t _rsvd;  /**< reserved */
  uint16_t rpm;   /**< measured RPM (0 if pump off) */
} pl_vacuum_status_t;

#ifdef __cplusplus
}
#endif
