"""main.py -- entry point wiring the full stack into the LandingFSM.

`--sim` swaps in MockMavlink / MockGimbal and a synthetic camera so the whole
package runs and lands with no hardware attached:

    python -m landing_sw.main --sim

Without --sim (on the Pi) it runs the October demo path, demo_vision.py: camera +
ESP32Gimbal, marker position printed in the body frame. No autopilot is needed;
arguments after `--` are passed through, e.g. `main -- --camera imx477`.

The sim integrates a simple drone kinematic over a stationary pad at the origin,
rendering each frame with synthetic_markers, and reports the touchdown residual.
"""

from __future__ import annotations

import argparse

import numpy as np

from . import config, ekf, fsm, landing, markers, pointing, synthetic_markers
from .gimbal_iface import MockGimbal
from .mavlink_iface import MockMavlink
from .telemetry_db import TelemetryDB


def run_sim(start_xyz=(1.2, -0.8, 6.0), max_t=40.0, verbose=True):
    rng = np.random.default_rng(0)
    detector = markers.MarkerDetector()
    kf = ekf.RelativeMarkerEKF()
    mav = MockMavlink(agl=start_xyz[2], rc="LAND")
    gimbal = MockGimbal()
    point = pointing.PointingController()
    land = landing.LandingController(mav)
    db = TelemetryDB()
    flight_id = db.start_flight("sim_descent", notes="main --sim closed loop")
    machine = fsm.LandingFSM(detector, kf, point, land, gimbal, mav, db, flight_id)

    p = np.array(start_xyz, dtype=np.float64)   # drone world pos over pad@origin
    dt = machine.dt
    n = int(max_t / dt)
    touched = False
    for _ in range(n):
        agl = max(p[2], 0.0)
        # Render through the live gimbal attitude so the synthetic image and the
        # camera->body transform share one convention (pitch+90 = tilt off nadir).
        gp, gr, gy = gimbal.get_angles()
        img, _ = synthetic_markers.render_view(agl, offset_xy=(p[0], p[1]),
                                               tilt_deg=gp + 90.0, yaw_deg=gy, rng=rng)
        mav.set_agl(agl)
        mav.set_drone_vel((mav.last_setpoint[0], mav.last_setpoint[1],
                           mav.last_setpoint[2]))
        machine.step(img, agl, rc="LAND", attitude=(0.0, 0.0, 0.0))
        gimbal.integrate(dt)
        vx, vy, vz = mav.last_setpoint
        # synthetic_markers renders image-right = world x, image-down = -world y, and at nadir
        # image-right = body Right, image-down = body Back (frames.R_LINK_OPTICAL). So with yaw 0
        # the sim world is x = Right, y = Forward, z = Up (T3c step 3: sim agrees with frames.py).
        p[0] += vy * dt
        p[1] += vx * dt
        p[2] -= vz * dt
        if agl <= config.FLARE_AGL_M and machine.ekf.initialized:
            touched = True
            resid = float(np.hypot(*machine.ekf.position[:2]))
            if verbose:
                print(f"  touchdown at AGL={agl:.2f} m, residual={resid*100:.1f} cm (EKF), "
                      f"{np.hypot(p[0], p[1])*100:.1f} cm (true), state={machine.state.value}")
            break
    db.commit()
    db.close()
    if verbose and not touched:
        print(f"  ended without flare: AGL={p[2]:.2f} m state={machine.state.value}")
    return {"touchdown": touched}


def main():
    ap = argparse.ArgumentParser(description="Vision precision-landing companion")
    ap.add_argument("--sim", action="store_true",
                    help="run with mock backends + synthetic camera (no hardware)")
    args, rest = ap.parse_known_args()
    if args.sim:
        run_sim()
    else:  # pragma: no cover - hardware path
        from . import demo_vision
        demo_vision.main([a for a in rest if a != "--"])


if __name__ == "__main__":
    main()
