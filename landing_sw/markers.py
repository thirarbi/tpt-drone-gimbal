"""markers.py -- MarkerDetector for the nested-ArUco landing pad.

detect(gray, est_agl_m) -> Pose. Detects the nested outer/inner ArUco pair via
cv2.aruco, selects which marker to trust from est_agl_m / HANDOFF_AGL_M and which
is actually visible (principle 5), solves the pose with SOLVEPNP_IPPE_SQUARE on
the 4 corners, and rejects poses whose reprojection error exceeds REPROJ_ERR_MAX_PX
-- the detector-side half of the pose-flip mitigation (principle 6; the EKF chi2
gate is the other half). last_reproj_err is exposed for logging.
"""

from __future__ import annotations

import cv2
import numpy as np

from . import config, synthetic_markers


def _square_obj(side: float) -> np.ndarray:
    h = side / 2.0
    return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float64)


class MarkerDetector:
    def __init__(self):
        self.K = config.cam_matrix()
        self.dist = config.cam_dist()
        self.dictionary = synthetic_markers.aruco_dictionary()
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, params)
        self.last_reproj_err = float("nan")

    def _detect_raw(self, gray):
        corners, ids, _ = self.detector.detectMarkers(gray)
        found = {}
        if ids is not None:
            for c, i in zip(corners, ids.flatten()):
                found[int(i)] = c.reshape(4, 2).astype(np.float64)
        return found

    @staticmethod
    def _reproj(obj, img, rvec, tvec, K, dist):
        proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
        return float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - img) ** 2, axis=1))))

    def detect(self, gray: np.ndarray, est_agl_m: float) -> config.Pose:
        found = self._detect_raw(gray)
        outer_vis = config.OUTER_MARKER_ID in found
        inner_vis = config.INNER_MARKER_ID in found

        use_inner = inner_vis and (est_agl_m <= config.HANDOFF_AGL_M or not outer_vis)
        if use_inner:
            mid, side = config.INNER_MARKER_ID, config.INNER_MARKER_SIDE_M
        elif outer_vis:
            mid, side = config.OUTER_MARKER_ID, config.OUTER_MARKER_SIDE_M
        elif inner_vis:
            mid, side = config.INNER_MARKER_ID, config.INNER_MARKER_SIDE_M
        else:
            self.last_reproj_err = float("nan")
            return config.Pose(valid=False)

        img_pts = found[mid]
        obj_pts = _square_obj(side)
        ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, self.K, self.dist,
                                      flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            self.last_reproj_err = float("nan")
            return config.Pose(valid=False, marker_id=mid)

        self.last_reproj_err = self._reproj(obj_pts, img_pts, rvec, tvec,
                                            self.K, self.dist)
        valid = self.last_reproj_err <= config.REPROJ_ERR_MAX_PX
        return config.Pose(t=tvec.reshape(3), rvec=rvec.reshape(3),
                           frame="camera", valid=valid, marker_id=mid)

    def detect_with_ambiguity(self, gray: np.ndarray):
        """Return both IPPE_SQUARE solutions for the inner marker (pose-flip study)."""
        found = self._detect_raw(gray)
        if config.INNER_MARKER_ID not in found:
            return None
        img_pts = found[config.INNER_MARKER_ID]
        obj_pts = _square_obj(config.INNER_MARKER_SIDE_M)
        n, rvecs, tvecs, _ = cv2.solvePnPGeneric(obj_pts, img_pts, self.K, self.dist,
                                                 flags=cv2.SOLVEPNP_IPPE_SQUARE)
        sols = []
        for k in range(n):
            err = self._reproj(obj_pts, img_pts, rvecs[k], tvecs[k], self.K, self.dist)
            sols.append({"rvec": rvecs[k].reshape(3), "tvec": tvecs[k].reshape(3),
                         "reproj_err_px": err})
        sols.sort(key=lambda s: s["reproj_err_px"])
        return sols
