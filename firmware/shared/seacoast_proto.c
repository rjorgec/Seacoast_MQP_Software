#include "seacoast_proto.h"

uint16_t sc_crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else             crc = (crc << 1);
        }
    }
    return crc;
}

size_t sc_build_frame(uint8_t type, const void *payload, uint8_t len,
                      uint8_t *out, size_t out_cap)
{
    const size_t total = 2 + 2 + (size_t)len + 2; // sync(2) + type+len(2) + payload + crc(2)
    if (!out || out_cap < total) return 0;
    if (len && !payload) return 0;

    out[0] = SC_SYNC0;
    out[1] = SC_SYNC1;
    out[2] = type;
    out[3] = len;

    for (uint8_t i = 0; i < len; i++) out[4 + i] = ((const uint8_t *)payload)[i];

    uint16_t crc = sc_crc16_ccitt(&out[2], (size_t)2 + (size_t)len); // type+len+payload
    out[4 + len] = (uint8_t)(crc & 0xFF);
    out[5 + len] = (uint8_t)((crc >> 8) & 0xFF);

    return total;
}

void sc_parser_init(sc_parser_t *p)
{
    if (!p) return;
    p->idx = 0;
    p->want_len = 0;
    p->in_sync = false;
}

static void reset(sc_parser_t *p)
{
    p->idx = 0;
    p->want_len = 0;
    p->in_sync = false;
}

bool sc_parser_feed(sc_parser_t *p, uint8_t byte, sc_frame_view_t *out_view)
{
    if (!p || !out_view) return false;

    // Sync hunt
    if (!p->in_sync) {
        if (p->idx == 0) {
            if (byte == SC_SYNC0) {
                p->buf[p->idx++] = byte;
            }
            return false;
        } else if (p->idx == 1) {
            if (byte == SC_SYNC1) {
                p->buf[p->idx++] = byte;
                p->in_sync = true;
            } else {
                // restart, maybe byte is new sync0
                p->idx = 0;
                if (byte == SC_SYNC0) p->buf[p->idx++] = byte;
            }
            return false;
        }
    }

    // Collect header/type/len/payload/crc
    p->buf[p->idx++] = byte;

    // Once we have type+len (bytes 2 and 3), we know payload length
    if (p->idx == 4) {
        p->want_len = p->buf[3];
        if (p->want_len > SC_MAX_PAYLOAD) {
            reset(p);
        }
        return false;
    }

    const size_t full_len = 2 + 2 + (size_t)p->want_len + 2;
    if (p->idx < full_len) return false;

    // Validate CRC
    const uint8_t type = p->buf[2];
    const uint8_t len  = p->buf[3];
    const uint16_t got = (uint16_t)p->buf[4 + len] | ((uint16_t)p->buf[5 + len] << 8);
    const uint16_t exp = sc_crc16_ccitt(&p->buf[2], (size_t)2 + (size_t)len);

    if (got != exp) {
        reset(p);
        return false;
    }

    out_view->type = type;
    out_view->len  = len;
    out_view->payload = &p->buf[4];

    // Prepare for next frame
    reset(p);
    return true;
}
