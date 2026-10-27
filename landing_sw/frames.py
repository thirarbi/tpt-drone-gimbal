"""frames.py -- coordinate transforms between camera, body (FRD) and NED.

Implements the geometry the perception->control chain needs. The camera hangs on
a SimpleBGC gimbal, so the camera->body transform uses the LIVE streamed gimbal
angles (pitch/roll/yaw), not a fixed mounting assumption -- otherwise the marker
position would be mis-transformed whenever the gimbal moves (supports the
pointing/landing chain in fsm.py).

Frames
------
Camera optical: x right, y down, z forward (bore). OpenCV convention.
Body FRD: x Forward, y Right, z Down.
NED: x North, y East, z Down.
Gimbal pitch: 0 deg = bore forward (horizon), -90 deg = bore straight down.
"""

from __future__ import annotations

import numpy as np

from . import gimbal_kinematics as gk


# def _Rx(a):
#     c, s = np.cos(a), np.sin(a)
#     return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], float)


# def _Ry(a):
#     c, s = np.cos(a), np.sin(a)
#     return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], float)


def _Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], float)

# PROVISIONAL: assumes the camera is mounted upright. Confirm with Pasha (T1),
# and physically in T18 (marker to the drone's right -> body y > 0).
R_LINK_OPTICAL = np.array([[0, 0, 1],
                           [1, 0, 0],
                           [0, 1, 0]], float)



def camera_to_body_rotation(gimbal_angles_deg) -> np.ndarray:
    """Rotation R such that v_body = R @ v_camera, for the given gimbal attitude.

    gimbal_angles_deg = (pitch, roll, yaw), pitch 0=bore forward, -90=bore down.
    Anchored on the verified nadir map and lifted toward forward by (pitch+90)
    about body Right; gimbal yaw about body Down and roll about body Forward.
    """
    pitch, roll, yaw = (np.deg2rad(a) for a in gimbal_angles_deg)
    return gk.fk([yaw, roll, pitch]) @ R_LINK_OPTICAL


def camera_point_to_body(t_cam: np.ndarray, gimbal_angles_deg) -> np.ndarray:
    """Transform a point from the camera optical frame to the body FRD frame."""
    R = camera_to_body_rotation(gimbal_angles_deg)
    return R @ np.asarray(t_cam, dtype=np.float64).reshape(3)


def body_to_ned(v_body: np.ndarray, yaw_deg: float) -> np.ndarray:
    """Rotate a body-FRD vector into NED given the vehicle yaw (heading, deg)."""
    return _Rz(np.deg2rad(yaw_deg)) @ np.asarray(v_body, dtype=np.float64).reshape(3)


def ned_to_body(v_ned: np.ndarray, yaw_deg: float) -> np.ndarray:
    """Rotate an NED vector into the body FRD frame given vehicle yaw (deg)."""
    return _Rz(np.deg2rad(yaw_deg)).T @ np.asarray(v_ned, dtype=np.float64).reshape(3)
