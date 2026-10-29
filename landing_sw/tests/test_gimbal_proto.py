"""C <-> Python protocol agreement (plan T13: test the silent part only).

golden_proto.txt is written by gimbal_fw/host_test/test_proto.c (make -C gimbal_fw/host_test).
"""
import math
import os
import struct

import pytest

from landing_sw import gimbal_proto as gp
from landing_sw.gimbal_iface import ESP32Gimbal

GOLDEN = os.path.join(os.path.dirname(__file__), "..", "..", "gimbal_fw", "host_test", "golden_proto.txt")


def _golden():
    frames = {}
    with open(GOLDEN) as f:
        for line in f:
            if line.startswith("#") or not line.strip():
                continue
            name, hexstr = line.split()
            frames[name] = bytes.fromhex(hexstr)
    return frames


def test_crc_check_value():
    assert gp.crc16_ccitt(b"123456789") == 0x29B1


def test_struct_sizes_match_c():
    assert gp.SET_TARGET.size == 13 and gp.TELEM.size == 31


def test_python_encodes_the_same_bytes_as_c():
    want = _golden()["set_target"]
    got = gp.encode_set_target(0x1234, gp.MODE_HOLD, -1.5707964, 0.1, 0.25)
    assert got == want


def test_python_decodes_c_telem():
    (msg_type, seq, payload), = gp.Parser().feed(_golden()["telem"])
    assert msg_type == gp.MSG_TELEM and seq == 42
    t = gp.decode_telem(payload)
    assert t["proto_version"] == gp.PROTO_VERSION and t["mode"] == gp.MODE_NADIR
    assert gp.flag_names(t["flags"]) == ["IMU_OK", "SERVOS_OK"]
    assert t["q_rad"] == pytest.approx((0.1, -0.2, -1.5))        # yaw, roll, pitch: order matters
    assert t["cam_roll_rad"] == pytest.approx(0.01) and t["cam_pitch_rad"] == pytest.approx(-1.55)
    assert t["loop_max_us"] == 321 and t["ack_seq"] == 7 and t["servo_err"] == (0, 0x80, 0)


def test_parser_resyncs_after_garbage_and_bad_crc():
    good = _golden()["telem"]
    bad = bytearray(good)
    bad[10] ^= 1
    p = gp.Parser()
    out = p.feed(b"\x00\xA5\x11\xA5\x5A\x01\xFF\xA5" + bytes(bad) + good)
    assert len(out) == 1 and p.crc_errors == 1


class FakeSerial:
    def __init__(self, rx=b""):
        self.rx = bytearray(rx)
        self.tx = bytearray()

    @property
    def in_waiting(self):
        return len(self.rx)

    def read(self, n):
        out, self.rx = bytes(self.rx[:n]), self.rx[n:]
        return out

    def write(self, b):
        self.tx += b

    def close(self):
        pass


def _telem_frame(q_rad, ack_seq=0):
    payload = gp.TELEM.pack(gp.PROTO_VERSION, gp.MODE_HOLD, 0, *q_rad, 0.0, -1.57, 100, ack_seq, 0, 0, 0)
    return gp.encode(gp.MSG_TELEM, 1, payload)


def test_get_angles_converts_joint_order_and_units():
    yaw, roll, pitch = 0.1, -0.2, -1.5
    g = ESP32Gimbal(ser=FakeSerial(_telem_frame((yaw, roll, pitch))), start=False)
    g.poll_once()
    assert g.get_angles() == pytest.approx(tuple(math.degrees(a) for a in (pitch, roll, yaw)))


def test_set_angles_sends_radians_in_hold_mode():
    ser = FakeSerial()
    g = ESP32Gimbal(ser=ser, start=False)
    g.set_angles(-90.0, 5.0, 10.0)
    (msg_type, seq, payload), = gp.Parser().feed(bytes(ser.tx))
    mode, p, r, y = gp.SET_TARGET.unpack(payload)
    assert msg_type == gp.MSG_SET_TARGET and mode == gp.MODE_HOLD
    assert (p, r, y) == pytest.approx((-math.pi / 2, math.radians(5), math.radians(10)), abs=1e-6)

    ser.rx += _telem_frame((0.0, 0.0, -1.57), ack_seq=seq)
    g.poll_once()
    assert g.latency_s >= 0.0
