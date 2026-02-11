#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---- Framing ----
// [0]=0xA5 [1]=0x5A [2]=type [3]=len [4..]=payload [end-2..end-1]=crc16(type+len+payload)
#define SC_SYNC0 0xA5
#define SC_SYNC1 0x5A
#define SC_MAX_PAYLOAD 255

typedef enum {
    SC_MSG_CMD_START  = 0x01,
    SC_MSG_CMD_STOP   = 0x02,
    SC_MSG_CMD_TARE   = 0x03,
    SC_MSG_CMD_CLEAN  = 0x04,
    SC_MSG_CMD_HOME   = 0x05,
    SC_MSG_CMD_PAUSE  = 0x06,

    SC_MSG_ACK        = 0x80,
    SC_MSG_STATUS     = 0x81,
} sc_msg_type_t;

typedef struct __attribute__((packed)) {
    float    target_g;
    uint16_t recipe_id;
} sc_cmd_start_t;

typedef struct __attribute__((packed)) {
    uint8_t  cmd_type;   // one of SC_MSG_CMD_*
    int8_t   result;     // 0=OK, negative=error
} sc_ack_t;

typedef struct __attribute__((packed)) {
    uint8_t  state;      // app-defined
    float    mass_g;     // if available/simulated
    uint16_t fault;      // 0=none
} sc_status_t;

// Build a frame into out[]. Returns total frame length, or 0 on error.
size_t sc_build_frame(uint8_t type, const void *payload, uint8_t len,
                      uint8_t *out, size_t out_cap);

// Parser: feed bytes in, emits a complete frame when available.
typedef struct {
    uint8_t  buf[4 + SC_MAX_PAYLOAD + 2];
    size_t   idx;
    uint8_t  want_len;     // payload len
    bool     in_sync;
} sc_parser_t;

typedef struct {
    uint8_t type;
    uint8_t len;
    const uint8_t *payload; // points into parser buffer
} sc_frame_view_t;

void sc_parser_init(sc_parser_t *p);

// Returns true when a full valid frame is decoded into *out_view (valid until next call).
bool sc_parser_feed(sc_parser_t *p, uint8_t byte, sc_frame_view_t *out_view);

// CRC16-CCITT (0x1021), init 0xFFFF.
uint16_t sc_crc16_ccitt(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif
