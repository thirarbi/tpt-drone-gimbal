// Pi protocol: CRC, framing, resync. Also writes golden_proto.txt, which
// landing_sw/tests/test_gimbal_proto.py decodes: that is what catches a C/Python mismatch
// (CRC convention, or swapped struct fields such as pitch <-> roll).
#include <stdio.h>
#include <string.h>
#include "proto.h"

static int fails;
#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)

static int feed_all(proto_parser_t *p, const uint8_t *b, size_t n, int *hits) {
    int r = 0;
    for (size_t i = 0; i < n; i++) {
        r = proto_parser_feed(p, b[i]);
        if (r == 1) (*hits)++;
    }
    return r;
}

static void write_hex(FILE *f, const char *name, const uint8_t *b, size_t n) {
    fprintf(f, "%s ", name);
    for (size_t i = 0; i < n; i++) fprintf(f, "%02X", b[i]);
    fprintf(f, "\n");
}

int main(void) {
    CHECK(crc16_ccitt((const uint8_t *)"123456789", 9) == 0x29B1);   // CCITT-FALSE check value
    CHECK(sizeof(msg_set_target_t) == 13);
    CHECK(sizeof(msg_telem_t) == 31);

    const msg_set_target_t st = { GMODE_HOLD, -1.5707964f, 0.1f, 0.25f };
    uint8_t f_st[PROTO_MAX_FRAME];
    size_t n_st = proto_encode(MSG_SET_TARGET, 0x1234, &st, sizeof st, f_st);
    CHECK(n_st == PROTO_OVERHEAD + sizeof st);

    const msg_telem_t tm = {
        .proto_version = PROTO_VERSION, .mode = GMODE_NADIR, .flags = GFLAG_IMU_OK | GFLAG_SERVOS_OK,
        .q_rad = {0.1f, -0.2f, -1.5f}, .cam_roll_rad = 0.01f, .cam_pitch_rad = -1.55f,
        .loop_max_us = 321, .ack_seq = 7, .servo_err = {0x00, 0x80, 0x00},
    };
    uint8_t f_tm[PROTO_MAX_FRAME];
    size_t n_tm = proto_encode(MSG_TELEM, 42, &tm, sizeof tm, f_tm);

    // 1. round trip
    proto_parser_t p;
    proto_parser_reset(&p);
    p.crc_errors = 0;
    int hits = 0;
    CHECK(feed_all(&p, f_st, n_st, &hits) == 1 && hits == 1);
    CHECK(proto_type(&p) == MSG_SET_TARGET && proto_seq(&p) == 0x1234 && proto_len(&p) == sizeof st);
    msg_set_target_t got;
    memcpy(&got, proto_payload(&p), sizeof got);
    CHECK(got.mode == GMODE_HOLD && got.pitch_rad == st.pitch_rad && got.roll_rad == st.roll_rad);

    // 2. garbage (incl. a lone A5 and an impossible length) before a good frame
    static const uint8_t junk[] = {0x00, 0xA5, 0x11, 0xA5, 0x5A, 0x01, 0xFF, 0xA5};
    hits = 0;
    feed_all(&p, junk, sizeof junk, &hits);
    CHECK(feed_all(&p, f_tm, n_tm, &hits) == 1 && hits == 1);
    CHECK(proto_type(&p) == MSG_TELEM && proto_seq(&p) == 42);

    // 3. one flipped payload bit: rejected and counted, and the next frame still parses
    uint8_t bad[PROTO_MAX_FRAME];
    memcpy(bad, f_st, n_st);
    bad[8] = (uint8_t)(bad[8] ^ 0x01u);
    hits = 0;
    feed_all(&p, bad, n_st, &hits);
    CHECK(hits == 0 && p.crc_errors == 1);
    CHECK(feed_all(&p, f_st, n_st, &hits) == 1 && hits == 1);

    FILE *f = fopen("golden_proto.txt", "w");
    CHECK(f != NULL);
    if (f) {
        fprintf(f, "# written by host_test/test_proto.c, read by landing_sw/tests/test_gimbal_proto.py\n");
        write_hex(f, "set_target", f_st, n_st);
        write_hex(f, "telem", f_tm, n_tm);
        fclose(f);
    }

    if (fails) { printf("test_proto: %d failure(s)\n", fails); return 1; }
    printf("test_proto: OK\n");
    return 0;
}
