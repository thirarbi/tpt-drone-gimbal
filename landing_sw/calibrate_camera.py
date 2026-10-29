"""calibrate_camera.py -- checkerboard calibration at detect_res (plan T18). Prints K and
dist to paste into config.py (CAM_K/CAM_DIST or IMX477_K/IMX477_DIST).

    python -m landing_sw.calibrate_camera --camera imx477 --cols 9 --rows 6 --square-m 0.025

cols x rows = INNER corners of the printed board. Move it around: corners of the image,
tilted ~30 deg, near and far. Lock focus on the lens first; refocusing changes K.
"""

from __future__ import annotations

import argparse
import time

import cv2
import numpy as np

from .camera import PROFILES, Camera


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--camera", choices=sorted(PROFILES), default="ov9281")
    ap.add_argument("--cols", type=int, default=9)
    ap.add_argument("--rows", type=int, default=6)
    ap.add_argument("--square-m", type=float, default=0.025)
    ap.add_argument("--views", type=int, default=20)
    ap.add_argument("--interval-s", type=float, default=1.0, help="min time between accepted views")
    args = ap.parse_args(argv)

    pattern = (args.cols, args.rows)
    objp = np.zeros((args.cols * args.rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * args.square_m
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)

    cam = Camera(args.camera)
    obj_pts, img_pts, last, gray = [], [], 0.0, None
    print(f"{cam.profile.name}: show the {args.cols}x{args.rows} board, {args.views} views")
    try:
        while len(img_pts) < args.views:
            gray, _ = cam.read_gray()
            ok, corners = cv2.findChessboardCorners(gray, pattern, cv2.CALIB_CB_FAST_CHECK)
            if ok and time.monotonic() - last > args.interval_s:
                img_pts.append(cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), crit))
                obj_pts.append(objp)
                last = time.monotonic()
                print(f"  view {len(img_pts)}/{args.views}: move the board")
    finally:
        cam.close()

    rms, K, dist, _, _ = cv2.calibrateCamera(obj_pts, img_pts, gray.shape[::-1], None, None)
    print(f"RMS reprojection error {rms:.3f} px (good: < 0.5)")
    print(f"K    = {np.round(K, 1).tolist()}")
    print(f"DIST = {np.round(dist.ravel(), 5).tolist()}")


if __name__ == "__main__":
    main()
