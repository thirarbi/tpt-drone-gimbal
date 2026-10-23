A companion computer is a device that travels on-board the vehicle and controls/communicates with the autopilot over a low-latency link. Apps running on a companion computer can perform computationally intensive or time-sensitive tasks, and add much greater intelligence than is provided by the autopilot alone.

e.g. Raspberry Pi.

In this project, `landing_sw/` is the **companion-computer software** for a precision-landing drone. The physical system currently is still under discussion. Here, I'm gonna make some assumptions to write the software that wouldn't be affected.

![[Pasted image 20260916112044.png]]

- **Sensor:** `markers.py` detects + `synthetic_markers.py` (the fake camera in sim)
- **Coordinate plumbing:** `frames.py` rotates the measurement out of the camera frame into the drone body frame.
- **Estimator:** `ekf.py` the specialized Kalman filter.
- **Controller:** `landing.py` is the vehicle velocity + `pointing.py` the gimbal.
- **Plant:** The drone, simulated by a kinematic integrator in `main.py / sitl_harness.py`, or real Arducopter via `mvlink_iface.py`.
- **Supervisor:** `fsm.py` decides *which* of the above run, based on marker visibility, altitude, and mode.