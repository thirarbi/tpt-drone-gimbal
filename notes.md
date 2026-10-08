`landing_sw/` is the **companion-computer software** for a precision-landing drone. The physical system currently is still under discussion. Here, I'm gonna make some assumptions to write the software that wouldn't be affected.

![[Pasted image 20260916112044.png]]

- **Sensor:** `markers.py` detects + `synthetic_markers.py` (the fake camera in sim)
- **Coordinate plumbing:** `frames.py` rotates the measurement out of the camera frame into the drone body frame.
- **Estimator:** `ekf.py` the specialized Kalman filter.
- **Controller:** `landing.py` is the vehicle velocity + `pointing.py` the gimbal.
- **Plant:** The drone, simulated by a kinematic integrator in `main.py / sitl_harness.py`, or real Arducopter via `mvlink_iface.py`.
- **Supervisor:** `fsm.py` decides *which* of the above run, based on marker visibility, altitude, and mode.

### The 7 Design Principles

1. Latency is the benchmark. The system budgets photon-to-setpoint latency (target 33 ms, ceiling 66 ms), not just camera FPS. It's set on `config.LATENCY_ALLOC_MS`, `latency_bench.py`. 
2. **Velocity feedforward**. `v_cmd = Kp.error G_ff.v_target_absolute`. Written in `landing.py`, `config.FEEDFORWARD_GAIN`.

### Reading `config.py`
There are no bare numeric literals anywhere else in the codebase, except here. Everything you tune is in this file, tagged with provenance:
- `# MEASURE`: Must be confirmed against real hardware / calibration (e.g. the camera matrix `CAM_K`,`EKF_MEAS_POS_STD`)
- `# TYPICAL`: Datasheet/handbook stand-in until measured.
- `# DESIGN`: Chosen design value.
