#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PROTO_VERSION      1u
#define PROTO_MAX_PAYLOAD  128u
#define PROTO_DELIM        0x00u

/*
 * Wire format endianness is little-endian for all multi-byte fields.
 * ESP32-C6 and RP2040 are both little-endian, so native packed structs
 * are used consistently on both sides.
 */

typedef enum {
    MSG_PING = 0x01,

    MSG_MOTOR_DRV8163_START_MON = 0x10,
    MSG_MOTOR_DRV8163_STOP_MON  = 0x11,

    MSG_MOTOR_STEPPER_ENABLE    = 0x20,
    MSG_MOTOR_STEPPER_STEPJOB   = 0x21,

    MSG_CTRL_TARE               = 0x30,
    MSG_CTRL_START              = 0x31,
    MSG_CTRL_STOP               = 0x32,
    MSG_CTRL_PAUSE              = 0x33,

    MSG_ACK                     = 0x80,
    MSG_NACK                    = 0x81,
} msg_type_t;

typedef enum {
    NACK_BAD_FRAME   = 1,
    NACK_BAD_CRC     = 2,
    NACK_BAD_LEN     = 3,
    NACK_BAD_VERSION = 4,
    NACK_UNKNOWN     = 5,
} nack_code_t;

typedef struct __attribute__((packed)) {
    uint8_t  version;
    uint8_t  type;
    uint16_t seq;
    uint16_t len;
} proto_hdr_t;

typedef struct __attribute__((packed)) {
    uint16_t speed;
    uint16_t low_th;
    uint16_t high_th;
    uint32_t interval_ms;
} pl_drv8163_start_mon_t;

typedef struct __attribute__((packed)) {
    uint8_t enable;
} pl_stepper_enable_t;

typedef struct __attribute__((packed)) {
    uint8_t  dir;
    uint32_t steps;
    uint32_t step_delay_us;
} pl_stepper_stepjob_t;

typedef struct __attribute__((packed)) {
    uint8_t code;
} pl_nack_t;

uint16_t proto_crc16_ccitt(const uint8_t *data, uint32_t len);

#ifdef __cplusplus
}
#endif
