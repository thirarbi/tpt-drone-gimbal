// Pi <-> ESP32 frame (plan §4.5). PURE C: host-tested, and mirrored in landing_sw/gimbal_proto.py.
// [0xA5][0x5A][type u8][len u8][seq u16][payload][crc u16], little-endian,
// crc = CRC-16/CCITT-FALSE over type..payload.
// One struct change = change BOTH sides + bump PROTO_VERSION.
#pragma once
#include <stddef.h>
#include <stdint.h>

#define PROTO_VERSION      1
#define PROTO_SYNC1        0xA5
#define PROTO_SYNC2        0x5A
#define PROTO_MAX_PAYLOAD  48
#define PROTO_OVERHEAD     8    // sync 2 + type + len + seq 2 + crc 2
#define PROTO_MAX_FRAME    (PROTO_MAX_PAYLOAD + PROTO_OVERHEAD)

#define MSG_SET_TARGET     0x01 // Pi -> ESP32
#define MSG_TELEM          0x81 // ESP32 -> Pi

#define GMODE_PARK         0    // servo torque off
#define GMODE_HOLD         1    // stabilise to the target
#define GMODE_NADIR        2    // stabilise to pitch -pi/2, roll 0, yaw_rel 0

// TELEM flags
#define GFLAG_IMU_OK       (1u << 0)
#define GFLAG_SERVOS_OK    (1u << 1)  // every present servo answered in the last cycle
#define GFLAG_QMEAS_STALE  (1u << 2)  // control law fell back to q_cmd
#define GFLAG_SINGULAR     (1u << 3)
#define GFLAG_WATCHDOG     (1u << 4)  // no valid Pi frame for COMMS_WATCHDOG_MS -> NADIR
#define GFLAG_SERVO_ERR    (1u << 5)  // a servo returned a non-zero error byte
#define GFLAG_NO_TORQUE    (1u << 6)  // torque refused (bring-up failed / voltage out of range)
#define GFLAG_AT_LIMIT     (1u << 7)
#define GFLAG_PID_MODE     (1u << 8)

typedef struct __attribute__((packed)) {    // Python: '<B3f'
    uint8_t mode;
    float   pitch_rad, roll_rad, yaw_rel_rad;
} msg_set_target_t;

typedef struct __attribute__((packed)) {    // Python: '<BBH3f2fHH3B'
    uint8_t  proto_version;
    uint8_t  mode;
    uint16_t flags;
    float    q_rad[3];        // MEASURED joint angles yaw, roll, pitch (body-relative)
    float    cam_roll_rad;    // camera attitude from the IMU (world)
    float    cam_pitch_rad;
    uint16_t loop_max_us;
    uint16_t ack_seq;         // seq of the last SET_TARGET received
    uint8_t  servo_err[3];    // last error byte per joint
} msg_telem_t;

uint16_t crc16_ccitt(const uint8_t *data, size_t n);
size_t   proto_encode(uint8_t type, uint16_t seq, const void *payload, uint8_t len, uint8_t *out);

typedef struct {
    uint8_t  buf[PROTO_MAX_FRAME];
    uint16_t n;
    uint32_t crc_errors;
} proto_parser_t;

void proto_parser_reset(proto_parser_t *p);
int  proto_parser_feed(proto_parser_t *p, uint8_t b);   // 1 = a valid frame is in buf, 0 = need more

// valid only after feed() returns 1:
static inline uint8_t        proto_type(const proto_parser_t *p)    { return p->buf[2]; }
static inline uint8_t        proto_len(const proto_parser_t *p)     { return p->buf[3]; }
static inline uint16_t       proto_seq(const proto_parser_t *p)     { return (uint16_t)(p->buf[4] | (p->buf[5] << 8)); }
static inline const uint8_t *proto_payload(const proto_parser_t *p) { return &p->buf[6]; }
