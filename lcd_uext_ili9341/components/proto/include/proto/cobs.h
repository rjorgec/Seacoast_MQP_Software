#pragma once
#include <stdint.h>
#include <stddef.h>

// returns encoded length
static inline size_t cobs_encode(const uint8_t *in, size_t len, uint8_t *out) {
    size_t r = 0, w = 1, code_i = 0;
    uint8_t code = 1;

    while (r < len) {
        if (in[r] == 0) {
            out[code_i] = code;
            code = 1;
            code_i = w++;
            r++;
        } else {
            out[w++] = in[r++];
            code++;
            if (code == 0xFF) {
                out[code_i] = code;
                code = 1;
                code_i = w++;
            }
        }
    }
    out[code_i] = code;
    return w;
}

// returns decoded length, or 0 on error
static inline size_t cobs_decode(const uint8_t *in, size_t len, uint8_t *out) {
    if (len == 0) return 0;
    size_t r = 0, w = 0;

    while (r < len) {
        uint8_t code = in[r++];
        if (code == 0) return 0;
        for (uint8_t i = 1; i < code; i++) {
            if (r >= len) return 0;
            out[w++] = in[r++];
        }
        if (code != 0xFF && r < len) out[w++] = 0;
    }
    return w;
}
