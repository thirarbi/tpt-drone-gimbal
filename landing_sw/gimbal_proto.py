"""gimbal_proto.py -- Pi <-> ESP32 frames. Mirrors gimbal_fw/components/proto/proto.h exactly.

[0xA5][0x5A][type u8][len u8][seq u16][payload][crc u16], little-endian,
crc = CRC-16/CCITT-FALSE over type..payload. One struct change = change BOTH
sides + bump PROTO_VERSION.
"""

from __future__ import annotations

import binascii
import struct

PROTO_VERSION = 1
SYNC = b"\xA5\x5A"
MAX_PAYLOAD = 48

MSG_SET_TARGET = 0x01
MSG_TELEM = 0x81

MODE_PARK, MODE_HOLD, MODE_NADIR = 0, 1, 2

# TELEM flags (GFLAG_* in proto.h)
FLAGS = {
    0: "IMU_OK", 1: "SERVOS_OK", 2: "QMEAS_STALE", 3: "SINGULAR", 4: "WATCHDOG",
    5: "SERVO_ERR", 6: "NO_TORQUE", 7: "AT_LIMIT", 8: "PID_MODE",
}

_HDR = struct.Struct("<BBH")              # type, len, seq
SET_TARGET = struct.Struct("<B3f")        # mode, pitch_rad, roll_rad, yaw_rel_rad
TELEM = struct.Struct("<BBH3f2fHH3B")     # see msg_telem_t


def crc16_ccitt(data: bytes) -> int:
    return binascii.crc_hqx(data, 0xFFFF)


def encode(msg_type: int, seq: int, payload: bytes) -> bytes:
    body = _HDR.pack(msg_type, len(payload), seq & 0xFFFF) + payload
    return SYNC + body + struct.pack("<H", crc16_ccitt(body))


def encode_set_target(seq: int, mode: int, pitch_rad: float, roll_rad: float,
                      yaw_rel_rad: float) -> bytes:
    return encode(MSG_SET_TARGET, seq, SET_TARGET.pack(mode, pitch_rad, roll_rad, yaw_rel_rad))


def decode_telem(payload: bytes) -> dict:
    v = TELEM.unpack(payload)
    return {
        "proto_version": v[0], "mode": v[1], "flags": v[2],
        "q_rad": v[3:6],                  # MEASURED joints: yaw, roll, pitch
        "cam_roll_rad": v[6], "cam_pitch_rad": v[7],
        "loop_max_us": v[8], "ack_seq": v[9], "servo_err": v[10:13],
    }


def flag_names(flags: int) -> list:
    return [name for bit, name in FLAGS.items() if flags & (1 << bit)]


class Parser:
    """Byte-wise, resyncs on garbage and bad CRC (same state machine as proto.c)."""

    def __init__(self):
        self.buf = bytearray()
        self.crc_errors = 0

    def feed(self, data: bytes) -> list:
        """Returns a list of (type, seq, payload) for every valid frame completed by data."""
        out = []
        for b in data:
            n = len(self.buf)
            if n == 0:
                if b == 0xA5:
                    self.buf.append(b)
                continue
            if n == 1:
                if b == 0x5A:
                    self.buf.append(b)
                elif b != 0xA5:
                    self.buf.clear()
                continue
            self.buf.append(b)
            if len(self.buf) == 4 and self.buf[3] > MAX_PAYLOAD:
                self.buf.clear()
                continue
            if len(self.buf) < 4 or len(self.buf) < 8 + self.buf[3]:
                continue
            frame, self.buf = bytes(self.buf), bytearray()
            length = frame[3]
            body = frame[2:6 + length]
            if crc16_ccitt(body) != struct.unpack_from("<H", frame, 6 + length)[0]:
                self.crc_errors += 1
                continue
            msg_type, _, seq = _HDR.unpack_from(body)
            out.append((msg_type, seq, body[4:]))
        return out
