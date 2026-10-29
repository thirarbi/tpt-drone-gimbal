"""config.py -- single source of truth for the vision-landing companion software.

Every other module imports its tunables from here; there are no bare numeric
literals elsewhere. Hardware-dependent figures are tagged inline:
    # MEASURE  -> must be confirmed against real hardware/calibration.
    # TYPICAL  -> datasheet / handbook stand-in until measured.

System: Raspberry Pi 5 companion + ArduCam OV9281 global-shutter mono camera
(ArduCam HQ IMX477 as a test alternative, see camera.py) + 3-axis XL-320 gimbal
driven by an ESP32 (gimbal_fw) + SpeedyBee F405 (ArduCopter) over MAVLink/pymavlink. The
companion detects a nested-ArUco landing pad and commands a precision landing,
including onto a moving target (ITB Teknik Fisika TF3001).
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field

import numpy as np

# --------------------------------------------------------------------------- #
# Paths
# --------------------------------------------------------------------------- #
PKG_DIR = os.path.dirname(os.path.abspath(__file__))
RESULTS_DIR = os.path.join(PKG_DIR, "results")


def results_path(name: str) -> str:
    """Absolute path inside results/, creating the directory on demand."""
    os.makedirs(RESULTS_DIR, exist_ok=True)
    return os.path.join(RESULTS_DIR, name)


# --------------------------------------------------------------------------- #
# Camera (principle 7: lo-res Y stream detects, hi-res stream records)
# --------------------------------------------------------------------------- #
CAM_NAME            = "ArduCam OV9281 (mono, global shutter)"
CAM_FULL_RES        = (1280, 800)
CAM_FPS             = 30
DETECT_RES          = (640, 400)        # lo-res Y detection stream
RECORD_RES          = CAM_FULL_RES      # hi-res telemetry stream
CAM_K  = [[560, 0, 320], [0, 560, 200], [0, 0, 1]]   # MEASURE (for DETECT_RES)
CAM_DIST = [0, 0, 0, 0, 0]                            # MEASURE k1,k2,p1,p2,k3

# ArduCam HQ (IMX477): test alternative to the OV9281 (camera.py --camera imx477).
# Colour + rolling shutter: the ISP's luma (Y) plane stands in for the mono image, and
# fast hand motion skews the marker more. 2028x1520 = 2x2-binned full field of view.
IMX477_NAME       = "ArduCam HQ IMX477 (colour, rolling shutter)"
IMX477_FULL_RES   = (2028, 1520)
IMX477_DETECT_RES = (640, 480)          # 4:3 like the sensor: a scale of full FOV, not a crop
IMX477_K    = [[611, 0, 320], [0, 611, 240], [0, 0, 1]]   # MEASURE: 6 mm lens estimate (16 mm: fx ~1630)
IMX477_DIST = [0, 0, 0, 0, 0]                              # MEASURE: run calibrate_camera.py

# --------------------------------------------------------------------------- #
# Nested ArUco (principle 5: outer at altitude, inner at low altitude)
# --------------------------------------------------------------------------- #
ARUCO_DICT          = "DICT_5X5_100"
OUTER_MARKER_ID     = 23
INNER_MARKER_ID     = 7
OUTER_MARKER_SIDE_M = 1.250   # sized so it clears ~23 px at AGL_MAX (30 m) on DETECT_RES
INNER_MARKER_SIDE_M = 0.100   # sized to bridge the handoff yet fit in FOV at AGL_MIN (0.3 m)
HANDOFF_AGL_M       = 0.90              # below this, trust inner marker only

# Detector reprojection-error gate (principle 6); px in DETECT_RES.
REPROJ_ERR_MAX_PX   = 3.0               # DESIGN

# --------------------------------------------------------------------------- #
# Latency budget (principle 1: photon-to-setpoint is the benchmark) -- ms
# --------------------------------------------------------------------------- #
LATENCY_TARGET_MS   = 33.0
LATENCY_BUDGET_MS   = 66.0
LATENCY_ALLOC_MS    = {                 # all MEASURE on Pi 5; sum must be <= ceiling
    "capture_exposure_readout": 12, "detect_aruco": 14, "solve_pnp": 4,
    "frame_transform": 1, "ekf_update": 2, "pointing+setpoint": 3,
    "mavlink_serial_tx": 4}

# --------------------------------------------------------------------------- #
# EKF (principle 3: 6-state, relative position + target ABSOLUTE velocity, body)
# --------------------------------------------------------------------------- #
EKF_RATE_HZ         = 100.0
EKF_PROC_ACCEL_STD  = 1.5              # m/s^2
EKF_MEAS_POS_STD    = 0.04            # m   # MEASURE
EKF_INIT_POS_STD    = 1.0            # m
EKF_INIT_VEL_STD    = 0.5            # m/s
EKF_GATE_CHI2       = 16.27          # chi^2, 3 dof, ~99.9%

# --------------------------------------------------------------------------- #
# FSM thresholds (principle 4: companion only bridges 30 fps gaps; EK3 dead-reckons)
# --------------------------------------------------------------------------- #
MARKER_LOSS_COAST_S = 2.0
GIMBAL_SEARCH_MIN_AGL = 2.5
GIMBAL_SEARCH_MAX_AGL = 30.0
FLARE_AGL_M         = 0.28
ABORT_CLIMB_RATE_MS = 1.5

# --------------------------------------------------------------------------- #
# Pointing / gimbal
# --------------------------------------------------------------------------- #
GIMBAL_PITCH_NAV_DEG  = 0.0
GIMBAL_PITCH_LAND_DEG = -90.0
GIMBAL_SLEW_DEG_S     = 60.0
POINTING_RATE_HZ      = 50.0

# --------------------------------------------------------------------------- #
# Landing controller (principle 2: v_cmd = Kp.err + Gff.v_target_absolute)
# --------------------------------------------------------------------------- #
LAND_RATE_HZ        = 20.0
KP_HORIZ            = 1.2
KP_VERT             = 0.8
FEEDFORWARD_GAIN    = 1.0
MAX_HORIZ_VEL_MS    = 4.0
MAX_DESCENT_VEL_MS  = 1.0
DESCENT_VEL_MS      = 0.6

# --------------------------------------------------------------------------- #
# BEC
# --------------------------------------------------------------------------- #
BEC_RATED_CURRENT_A = 5.0             # MEASURE -- verify against real BEC

# --------------------------------------------------------------------------- #
# Links
# --------------------------------------------------------------------------- #
MAVLINK_CONN       = "udp:127.0.0.1:14550"   # ArduPilot SITL
MAVLINK_BAUD       = 921600                   # hardware UART
ESP32_GIMBAL_PORT  = "/dev/ttyAMA0"   # Pi 5 GPIO14/15; serial0 may be the debug header
ESP32_GIMBAL_BAUD  = 115200           # = PI_UART_BAUD in gimbal_fw config_defaults.h
ESP32_GIMBAL_TX_HZ = 25.0             # SET_TARGET resend; firmware watchdog is 200 ms
TELEMETRY_DB_PATH  = "flightlog.sqlite"

# --------------------------------------------------------------------------- #
# Experiment acceptance thresholds (cited by the Bab 4.2 risk table)
# --------------------------------------------------------------------------- #
DETECTION_RATE_MIN      = 0.95     # row 2: min detection rate per AGL band
MOTION_BLUR_MAX_ERR_PX  = 1.0      # row 3: max rolling-shutter corner error (px)
POSE_FLIP_GATE_REJECT_MIN = 0.90   # row 4: min chi2 rejection of wrong PnP branch
POSE_FLIP_MAX_POS_IMPACT_CM = 5.0  # row 4: max landing-position error from a pose flip
FEEDFORWARD_MAX_STEADY_ERR_CM = 10.0   # row 5: max steady tracking error, FF on
BEC_MARGIN_MIN_A        = 1.0      # row (BEC): required headroom vs rated current

# Raspberry Pi 5 + payload 5 V-rail current model (used by exp_bec_current).
PI5_IDLE_A          = 0.85         # TYPICAL: Pi 5 idle @5V
PI5_CAMERA_A        = 0.35         # TYPICAL: CSI camera streaming adder
PI5_CPU_FULL_A      = 1.60         # TYPICAL: 4-core 100% load adder
PI5_WIFI_A          = 0.30         # TYPICAL: Wi-Fi TX burst adder
PI5_GIMBAL_SERIAL_A = 0.10         # TYPICAL: UART converter adder


def cam_matrix() -> np.ndarray:
    return np.array(CAM_K, dtype=np.float64)


def cam_dist() -> np.ndarray:
    return np.array(CAM_DIST, dtype=np.float64)


@dataclass
class Pose:
    """A 6-DOF pose estimate of a marker (or the camera w.r.t. a marker).

    t      : translation xyz in metres.
    rvec   : Rodrigues rotation vector.
    frame  : the frame t/rvec are expressed in ("camera" or "body").
    valid  : whether this pose is usable.
    marker_id : which marker produced it (-1 = none).
    """
    t: np.ndarray = field(default_factory=lambda: np.zeros(3))
    rvec: np.ndarray = field(default_factory=lambda: np.zeros(3))
    frame: str = "camera"
    valid: bool = False
    marker_id: int = -1
