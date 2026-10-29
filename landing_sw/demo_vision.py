"""demo_vision.py -- T18 hand-held marker demo: camera -> ArUco pose -> body frame
using the LIVE measured gimbal angles from the ESP32.

    python -m landing_sw.demo_vision --camera imx477             # ESP32 on /dev/ttyAMA0
    python -m landing_sw.demo_vision --camera imx477 --no-gimbal # camera only, gimbal assumed at nadir

The gimbal is held at NADIR: demo scope (§1) does not ask it to follow the marker.
Sign check (T3c/T18): marker to the drone's RIGHT -> y > 0, FORWARD -> x > 0, below -> z > 0.
Print the inner marker (ID 7) at exactly config.INNER_MARKER_SIDE_M, or the distances scale wrong.
"""

from __future__ import annotations

import argparse
import math
import time

from . import config, frames, markers
from . import gimbal_proto as gp
from .camera import PROFILES, Camera
from .gimbal_iface import ESP32Gimbal, MockGimbal

PRINT_EVERY_S = 0.2
STATUS_EVERY_S = 2.0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--camera", choices=sorted(PROFILES), default="ov9281")
    ap.add_argument("--port", default=config.ESP32_GIMBAL_PORT)
    ap.add_argument("--no-gimbal", action="store_true", help="no ESP32: assume the camera is at nadir")
    ap.add_argument("--agl", type=float, default=0.5,
                    help="assumed height (m); below HANDOFF_AGL_M the inner marker is preferred")
    args = ap.parse_args(argv)

    cam = Camera(args.camera)
    det = markers.MarkerDetector()
    det.K, det.dist = cam.profile.cam_matrix(), cam.profile.cam_dist()
    gimbal = MockGimbal() if args.no_gimbal else ESP32Gimbal(args.port)
    if isinstance(gimbal, ESP32Gimbal):
        gimbal.nadir()
    print(f"{cam.profile.name} @ {cam.profile.detect_res[0]}x{cam.profile.detect_res[1]}, "
          f"gimbal: {'none (nadir assumed)' if args.no_gimbal else args.port}.  Ctrl+C to stop.")

    n = n_det = 0
    lat_sum = 0.0
    t_status = t_print = time.monotonic()
    try:
        while True:
            gray, age = cam.read_gray()
            t0 = time.monotonic()
            pose = det.detect(gray, est_agl_m=args.agl)
            latency = age + (time.monotonic() - t0)     # photon -> pose (Bab 4.2 row 1)
            n += 1
            if pose.valid:
                n_det += 1
                if not math.isnan(latency):
                    lat_sum += latency
                now = time.monotonic()
                if now - t_print >= PRINT_EVERY_S:
                    t_print = now
                    g = gimbal.get_angles()
                    p = frames.camera_point_to_body(pose.t, g)
                    print(f"id {pose.marker_id:2d}  body x {p[0]:+.3f}  y {p[1]:+.3f}  z {p[2]:+.3f} m"
                          f"  | reproj {det.last_reproj_err:.2f} px | gimbal p/r/y "
                          f"{g[0]:+6.1f} {g[1]:+5.1f} {g[2]:+5.1f} deg | {latency * 1e3:.0f} ms")

            now = time.monotonic()
            if now - t_status >= STATUS_EVERY_S:
                fps = n / (now - t_status)
                line = (f"-- {fps:.1f} fps, detected {n_det}/{n}"
                        f", mean latency {lat_sum / n_det * 1e3:.0f} ms" if n_det else
                        f"-- {fps:.1f} fps, no marker (ID {config.INNER_MARKER_ID} or {config.OUTER_MARKER_ID}, "
                        f"{config.ARUCO_DICT})")
                if isinstance(gimbal, ESP32Gimbal):
                    t = gimbal.telem()
                    line += (" | gimbal: NO TELEM" if t is None else
                             f" | gimbal flags {','.join(gp.flag_names(t['flags'])) or '-'}")
                print(line)
                n = n_det = 0
                lat_sum = 0.0
                t_status = now
    except KeyboardInterrupt:
        pass
    finally:
        cam.close()
        if isinstance(gimbal, ESP32Gimbal):
            gimbal.close()


if __name__ == "__main__":
    main()
