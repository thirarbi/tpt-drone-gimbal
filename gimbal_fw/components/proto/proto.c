#include "proto.h"
#include <string.h>

// Poly 0x1021, init 0xFFFF, no reflection, no final xor. Python: binascii.crc_hqx(data, 0xFFFF)
uint16_t crc16_ccitt(const uint8_t *data, size_t n) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < n; i++) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

size_t proto_encode(uint8_t type, uint16_t seq, const void *payload, uint8_t len, uint8_t *out) {
    if (len > PROTO_MAX_PAYLOAD) return 0;
    out[0] = PROTO_SYNC1;
    out[1] = PROTO_SYNC2;
    out[2] = type;
    out[3] = len;
    out[4] = (uint8_t)(seq & 0xFF);
    out[5] = (uint8_t)(seq >> 8);
    if (len) memcpy(&out[6], payload, len);
    uint16_t crc = crc16_ccitt(&out[2], (size_t)(4 + len));
    out[6 + len] = (uint8_t)(crc & 0xFF);
    out[7 + len] = (uint8_t)(crc >> 8);
    return (size_t)(PROTO_OVERHEAD + len);
}

void proto_parser_reset(proto_parser_t *p) {
    p->n = 0;
}

int proto_parser_feed(proto_parser_t *p, uint8_t b) {
    if (p->n == 0) {
        if (b == PROTO_SYNC1) p->buf[p->n++] = b;
        return 0;
    }
    if (p->n == 1) {
        if (b == PROTO_SYNC2) p->buf[p->n++] = b;
        else p->n = (b == PROTO_SYNC1) ? 1 : 0;   // A5 A5 5A: the second A5 may be the real start
        return 0;
    }

    p->buf[p->n++] = b;
    if (p->n == 4 && p->buf[3] > PROTO_MAX_PAYLOAD) {   // impossible length: garbage, resync
        p->n = 0;
        return 0;
    }
    if (p->n < 4 || p->n < PROTO_OVERHEAD + p->buf[3]) return 0;

    uint16_t len = p->buf[3];
    uint16_t rx  = (uint16_t)(p->buf[6 + len] | (p->buf[7 + len] << 8));
    p->n = 0;   // buf stays valid until the next feed()
    if (crc16_ccitt(&p->buf[2], (size_t)(4 + len)) != rx) {
        p->crc_errors++;
        return 0;
    }
    return 1;
}
