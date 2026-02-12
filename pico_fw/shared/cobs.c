#include "cobs.h"

size_t cobs_encode(const uint8_t *in, size_t len, uint8_t *out) {
    const uint8_t *start = out;
    uint8_t *code_ptr = out++;
    uint8_t code = 1;

    for (size_t i = 0; i < len; i++) {
        if (in[i] == 0) {
            *code_ptr = code;
            code_ptr = out++;
            code = 1;
        } else {
            *out++ = in[i];
            code++;
            if (code == 0xFF) {
                *code_ptr = code;
                code_ptr = out++;
                code = 1;
            }
        }
    }
    *code_ptr = code;
    return (size_t)(out - start);
}

size_t cobs_decode(const uint8_t *in, size_t len, uint8_t *out) {
    const uint8_t *start = out;

    while (len) {
        uint8_t code = *in++;
        if (code == 0 || code > len + 1) return 0; // invalid
        len--;

        for (uint8_t i = 1; i < code; i++) {
            if (!len) return 0;
            *out++ = *in++;
            len--;
        }
        if (code != 0xFF && len) *out++ = 0;
    }

    return (size_t)(out - start);
}
