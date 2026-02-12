#include "proto.h"

uint16_t proto_crc16_ccitt(const uint8_t *data, uint32_t len) {
    uint16_t crc = 0xFFFFu;

    for (uint32_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
        }
    }

    return crc;
}
