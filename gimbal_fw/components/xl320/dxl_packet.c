#include "dxl_packet.h"

uint16_t crc16_dxl(const uint8_t *data, size_t n) {
    uint16_t crc = 0;
    for (size_t i = 0; i < n; i++) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x8005u) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

size_t dxl_build(uint8_t id, uint8_t inst, const uint8_t *params, uint16_t n, uint8_t *out)
{
    uint16_t len = (uint16_t)(n + 3);
    out[0] = 0xFF; out[1] = 0xFF; out[2] = 0xFD; out[3] = 0x00;
    out[4] = id;
    out[5] = (uint8_t)(len & 0xFF);
    out[6] = (uint8_t)(len >> 8);
    out[7] = inst;
    for (uint16_t i = 0; i < n; i++) out[8 + i] = params[i];
    uint16_t crc = crc16_dxl(out, (size_t)(8 + n));
    out[8 + n] = (uint8_t)(crc & 0xFF);
    out[9 + n] = (uint8_t)(crc >> 8);
    return (size_t)(10 + n);
}

void dxl_parser_reset(dxl_parser_t *p)
{
    p->n = 0;
}

int dxl_parser_feed(dxl_parser_t *p, uint8_t b)
{
    static const uint8_t hdr[4] = {0xFF, 0xFF, 0xFD, 0x00};
    if (p->n < 4) {
        if (b == hdr[p->n]) {
            p->buf[p->n++] = b;
        }
        // resync. FF FF FF FD: a third FF is still "FF FF so far", not a fresh start
        else {
            p->n = (b == 0xFF) ? (p->n == 2 ? 2 : 1) : 0;
        }
        return 0;
    }

    // overlong LEN: garbage
    if (p->n >= DXL_MAX_PKT) { 
        p->n = 0; return 0; 
    }

    p->buf[p->n++] = b;
    
    if (p->n < 7) {
        return 0;
    }

    uint16_t len = (uint16_t)(p->buf[5] | (p->buf[6] << 8));
    
    // status needs INST+ERR+CRC(2) at least
    if (len < 4 || 7u + len > DXL_MAX_PKT) { 
        p->n = 0;
        return 0;
    }

    if (p->n < 7u + len) {
        return 0;
    }

    // complete
    uint16_t rx  = (uint16_t)(p->buf[p->n - 2] | (p->buf[p->n - 1] << 8));
    uint16_t cal = crc16_dxl(p->buf, (size_t)(p->n - 2));
    uint8_t  inst = p->buf[7];

    // buf stays valid until the next feed()
    p->n = 0;
    if (cal != rx)               return -1;

    // echo of our own instruction
    if (inst != DXL_INST_STATUS) return 0;
    return 1;
}
