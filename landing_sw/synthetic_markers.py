"""synthetic_markers.py -- render the nested-ArUco pad at a known pose.

render_view(agl, offset_xy, tilt_deg, yaw_deg) -> (gray_image, ground_truth_pose)
renders the concentric outer+inner ArUco pad as the DETECT_RES camera would see
it from a given pose, returning both the image (for the genuine cv2 pipeline) and
the ground-truth marker pose (so perception can be scored against truth). The
small tilt/yaw perturbation is required by the pose-flip and detection-rate
experiments.

The pad is physically nested (principle 5): a large outer marker with a small
inner marker concentric in its centre on a white quiet ring. The outer dominates
detection at altitude; the inner takes over up close once the outer overflows the
FOV. Swapping render_view() for a real camera frame is a drop-in change -- every
consumer takes a plain HxW uint8 image.
"""

from __future__ import annotations

from functools import lru_cache

import cv2
import numpy as np

from . import config

_W, _H = config.DETECT_RES
INNER_QUIET_FRAC = 0.30        # white ring around inner marker (frac of inner side)


def aruco_dictionary():
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, config.ARUCO_DICT))


@lru_cache(maxsize=2)
def _pad_board(px: int = 1200) -> np.ndarray:
    """Render the nested pad to a px*px image spanning OUTER_MARKER_SIDE_M."""
    d = aruco_dictionary()
    border = max(8, px // 12)
    inner_region = px - 2 * border
    outer = cv2.aruco.generateImageMarker(d, config.OUTER_MARKER_ID, inner_region)
    canvas = np.full((px, px), 255, np.uint8)
    canvas[border:border + inner_region, border:border + inner_region] = outer

    frac = config.INNER_MARKER_SIDE_M / config.OUTER_MARKER_SIDE_M
    iside = int(round(frac * inner_region))
    pad = max(1, int(INNER_QUIET_FRAC * iside))
    inner = cv2.aruco.generateImageMarker(d, config.INNER_MARKER_ID, iside)
    c, half = px // 2, iside // 2
    canvas[c - half - pad:c + half + pad, c - half - pad:c + half + pad] = 255
    canvas[c - half:c - half + iside, c - half:c - half + iside] = inner
    return canvas


# --------------------------------------------------------------------------- #
def _Ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], float)


def _Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], float)


_R_NADIR = np.diag([1.0, -1.0, -1.0])


def camera_extrinsic(agl_m, offset_xy=(0.0, 0.0), tilt_deg=0.0, yaw_deg=0.0,
                     tilt_azimuth_deg=0.0):
    """(R_wc, t_wc) for a downward-looking camera over a pad at the world origin."""
    C = np.array([offset_xy[0], offset_xy[1], agl_m], float)
    R_wc = (_Rz(np.deg2rad(yaw_deg)) @ _Ry(np.deg2rad(tilt_deg))
            @ _Rz(np.deg2rad(tilt_azimuth_deg)) @ _R_NADIR)
    t_wc = -R_wc @ C
    return R_wc, t_wc


def project(points_world, R_wc, t_wc):
    """Project Nx3 world points to Nx2 px; points behind the camera -> NaN."""
    P = np.atleast_2d(points_world).astype(np.float64)
    K = config.cam_matrix()
    p_cam = (R_wc @ P.T + t_wc.reshape(3, 1)).T
    z = p_cam[:, 2]
    out = np.full((P.shape[0], 2), np.nan)
    v = z > 1e-6
    proj = (K @ p_cam[v].T).T
    out[v] = proj[:, :2] / proj[:, 2:3]
    return out


def outer_world_corners():
    h = config.OUTER_MARKER_SIDE_M / 2.0
    return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], float)


def render_view(agl_m, offset_xy=(0.0, 0.0), tilt_deg=0.0, yaw_deg=0.0,
                tilt_azimuth_deg=0.0, noise_sigma=2.0, rng=None):
    """Render the pad. Returns (gray_uint8 HxW, ground_truth Pose in camera frame)."""
    R_wc, t_wc = camera_extrinsic(agl_m, offset_xy, tilt_deg, yaw_deg, tilt_azimuth_deg)
    board = _pad_board()
    bpx = board.shape[0]
    dst = project(outer_world_corners(), R_wc, t_wc)

    img = np.full((_H, _W), 235, np.uint8)
    if np.all(np.isfinite(dst)):
        src = np.array([[0, 0], [bpx, 0], [bpx, bpx], [0, bpx]], np.float32)
        Hm = cv2.getPerspectiveTransform(src, dst.astype(np.float32))
        warp = cv2.warpPerspective(board, Hm, (_W, _H), borderValue=255)
        mask = np.zeros((_H, _W), np.uint8)
        cv2.fillConvexPoly(mask, dst.astype(np.int32), 255)
        img = np.where(mask > 0, warp, img)

    if noise_sigma > 0:
        rng = rng or np.random.default_rng()
        img = np.clip(img.astype(np.float64) + rng.normal(0, noise_sigma, img.shape),
                      0, 255).astype(np.uint8)

    # Ground-truth pad pose in camera frame: pad origin (0,0,0) -> t_wc.
    rvec, _ = cv2.Rodrigues(R_wc)
    gt = config.Pose(t=t_wc.reshape(3).copy(), rvec=rvec.reshape(3).copy(),
                     frame="camera", valid=True, marker_id=config.OUTER_MARKER_ID)
    return img, gt
