#pragma once
#include <stddef.h>
#include <stdint.h>

#define DXL_INST_PING        0x01
#define DXL_INST_READ        0x02
#define DXL_INST_WRITE       0x03
#define DXL_INST_REBOOT      0x08
#define DXL_INST_STATUS      0x55
#define DXL_INST_SYNC_WRITE  0x83
#define DXL_BROADCAST_ID     0xFE

// The largest packet we send or accept
// sync write of 3 = 10 + 4 + 9
#define DXL_MAX_PKT          32

uint16_t crc16_dxl(const uint8_t *data, size_t n);

// INVARIANT: params must never contain FF FF FD (no byte stuffing). Holds for ticks <= 1023, ids 1..3.
size_t dxl_build(uint8_t id, uint8_t inst, const uint8_t *params, uint16_t n, uint8_t *out);

typedef struct {
    uint8_t  buf[DXL_MAX_PKT];
    uint16_t n; // bytes collected so far
} dxl_parser_t;

void dxl_parser_reset(dxl_parser_t *p);
int dxl_parser_feed(dxl_parser_t *p, uint8_t b);
//  0 = need more
//  1 = valid STATUS
// -1 = bad CRC

// valid only after feed() returns 1:
static inline uint8_t        dxl_status_id(const dxl_parser_t *p) { return p->buf[4]; }
static inline uint8_t        dxl_status_err(const dxl_parser_t *p) { return p->buf[8]; }
static inline const uint8_t *dxl_status_data(const dxl_parser_t *p){ return &p->buf[9]; }
static inline uint16_t       dxl_status_ndata(const dxl_parser_t *p){ return (uint16_t)((p->buf[5] | (p->buf[6] << 8)) - 4); } // LEN - INST - ERR - CRC