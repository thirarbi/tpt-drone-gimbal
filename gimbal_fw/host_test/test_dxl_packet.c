#include <stdio.h>
#include <string.h>
#include "dxl_packet.h"

static int fails;
#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)

// Feeds every byte; returns the result of the LAST feed. Counts hits (== 1) in *hits.
static int feed_all(dxl_parser_t *p, const uint8_t *b, size_t n, int *hits)
{
    int r = 0;
    for (size_t i = 0; i < n; i++) {
        r = dxl_parser_feed(p, b[i]);
        if (r == 1 && hits) (*hits)++;
    }
    return r;
}

int main(void)
{
    // 1. CRC against the published check value
    CHECK(crc16_dxl((const uint8_t *)"123456789", 9) == 0xFEE8);

    // 2. PING builds the e-manual example byte-for-byte
    static const uint8_t want_ping[10] = {0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x03, 0x00, 0x01, 0x19, 0x4E};
    uint8_t ping[16];
    size_t n = dxl_build(1, DXL_INST_PING, NULL, 0, ping);
    CHECK(n == 10);
    CHECK(memcmp(ping, want_ping, 10) == 0);

    // A status packet is built the same way, with INST = 0x55 and params = ERR + data
    static const uint8_t st_params[3] = {0x00, 0x00, 0x02};   // ERR=0, data = 0x0200 (tick 512)
    uint8_t st[16];
    size_t sn = dxl_build(1, DXL_INST_STATUS, st_params, 3, st);

    // 3. echo + reply -> exactly one hit, and it is the reply
    dxl_parser_t p;
    dxl_parser_reset(&p);
    int hits = 0;
    feed_all(&p, ping, n, &hits);
    CHECK(hits == 0);                                          // our own echo is dropped
    int last = feed_all(&p, st, sn, &hits);
    CHECK(hits == 1 && last == 1);
    CHECK(dxl_status_id(&p) == 1);
    CHECK(dxl_status_err(&p) == 0);
    CHECK((dxl_status_data(&p)[0] | (dxl_status_data(&p)[1] << 8)) == 512);

    // 4. one flipped data bit -> bad CRC (-1), never a hit
    uint8_t bad[16];
    memcpy(bad, st, sn);
    bad[9] = (uint8_t)(bad[9] ^ 0x01u);
    hits = 0;
    CHECK(feed_all(&p, bad, sn, &hits) == -1 && hits == 0);

    // 5. resync: garbage, then FF FF FF before a good packet (the FF FF FF FD case)
    static const uint8_t junk[] = {0x12, 0xFF, 0x34, 0xFF, 0xFF};
    hits = 0;
    feed_all(&p, junk, sizeof junk, &hits);
    CHECK(feed_all(&p, st, sn, &hits) == 1 && hits == 1);

    printf(fails ? "test_dxl_packet: %d FAILURES\n" : "test_dxl_packet: OK\n", fails);
    return fails ? 1 : 0;
}
