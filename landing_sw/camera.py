"""camera.py -- picamera2 capture of the lo-res luma (Y) stream for the detector (plan T15).

Two profiles share one code path: "ov9281" (project camera, mono global shutter)
and "imx477" (ArduCam HQ, for testing). The main stream sets the sensor mode and
field of view; the detector only ever sees the lo-res stream at detect_res, so K
must be calibrated at detect_res (calibrate_camera.py).

Pi only: picamera2 is imported on use, so the rest of landing_sw runs on a laptop.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np

from . import config


@dataclass(frozen=True)
class CameraProfile:
    name: str
    full_res: tuple
    detect_res: tuple
    K: list
    dist: list

    def cam_matrix(self) -> np.ndarray:
        return np.array(self.K, dtype=np.float64)

    def cam_dist(self) -> np.ndarray:
        return np.array(self.dist, dtype=np.float64)


PROFILES = {
    "ov9281": CameraProfile(config.CAM_NAME, config.CAM_FULL_RES, config.DETECT_RES,
                            config.CAM_K, config.CAM_DIST),
    "imx477": CameraProfile(config.IMX477_NAME, config.IMX477_FULL_RES, config.IMX477_DETECT_RES,
                            config.IMX477_K, config.IMX477_DIST),
}


class Camera:
    def __init__(self, profile: str = "ov9281", fps: float = config.CAM_FPS, camera_num: int = 0):
        from picamera2 import Picamera2   # Pi-only dependency
        self.profile = PROFILES[profile]
        self._cam = Picamera2(camera_num)
        cfg = self._cam.create_video_configuration(
            main={"size": self.profile.full_res, "format": "YUV420"},
            lores={"size": self.profile.detect_res, "format": "YUV420"},
            controls={"FrameRate": fps},
            buffer_count=4)
        self._cam.configure(cfg)
        self._cam.start()

    def read_gray(self):
        """Returns (gray uint8 HxW at detect_res, age_s). age_s = now - the sensor's frame
        timestamp (CLOCK_MONOTONIC on the Pi), NaN if the metadata is missing."""
        req = self._cam.capture_request()
        try:
            yuv = req.make_array("lores")            # YUV420: the first H rows are the Y plane
            t_ns = req.get_metadata().get("SensorTimestamp")
        finally:
            req.release()
        w, h = self.profile.detect_res
        age = (time.monotonic_ns() - t_ns) * 1e-9 if t_ns else float("nan")
        return np.ascontiguousarray(yuv[:h, :w]), age

    def close(self):
        self._cam.stop()
        self._cam.close()
