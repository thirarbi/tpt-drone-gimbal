# Module 06 — The Drone Stack: MAVLink, ArduPilot, and the Hardware Interfaces

> The ecosystem layer. To understand *what this software is talking to*, you need a
> working model of: the **autopilot** (ArduCopter on the F405), the **companion
> computer** (the Pi running this code), the **MAVLink** protocol between them, and
> the **gimbal** on its own serial link. This module also teaches a software pattern
> you'll see throughout — the **real/mock interface split** via Python `Protocol`s —
> which is what lets the whole system run on your laptop.

## Learning objectives

- Explain the division of labor between **ArduCopter** (flight controller) and the
  **companion computer** (this code), and where the boundary sits.
- Explain what **MAVLink** is and read the specific messages this code sends/receives.
- Explain the velocity-setpoint command (`SET_POSITION_TARGET_LOCAL_NED`,
  `MAV_FRAME_BODY_NED`, the type-mask) at a level where you could modify it.
- Explain the **`Protocol` + real/mock** pattern and why it matters for testability.
- Place the gimbal interface in the same pattern.

## Read, in this order

1. [`landing_sw/mavlink_iface.py`](../landing_sw/mavlink_iface.py) — `VehicleBackend`
   protocol, `MavlinkInterface` (real), `MockMavlink` (sim).
2. [`landing_sw/gimbal_iface.py`](../landing_sw/gimbal_iface.py) — same pattern for
   the gimbal.
3. The "Links" block of `config.py` (connection strings, bauds, ports).

## The autopilot / companion division of labor

This is the single most important systems concept of the module. ArduCopter
(running on the SpeedyBee F405) is a complete, tuned flight-control stack:

- It runs the **fast inner loops** — attitude/rate control over the motors at
  hundreds of Hz, plus its own state estimator, **EKF3**, fusing IMU/GPS/baro/etc.
- It accepts high-level commands (modes, position/velocity setpoints) over MAVLink.

The **companion computer** (Pi 5, this code) runs the **slow outer loop**: vision,
the marker EKF, and the landing law — and sends ArduCopter **velocity setpoints**.
It *never* touches motors. This is the cascaded architecture from Module 01, and the
boundary is principle 4: the companion **does not** run a high-rate vehicle filter —
it *reads* ArduCopter's EKF3 velocity and trusts EKF3 to dead-reckon the vehicle
during marker loss. The companion's only filter is the little 6-state marker EKF.

> Mental model: ArduCopter is the "spinal cord + reflexes"; the companion is the
> "visual cortex" that says *where to go*. Keep that boundary crisp — a lot of
> design decisions in this codebase follow from respecting it.

## MAVLink in one page

**MAVLink** is the lightweight binary messaging protocol drones use to talk between
autopilot, ground stations, and companions. Messages are small packed structs with
IDs (`HEARTBEAT`, `ATTITUDE`, `LOCAL_POSITION_NED`, `RANGEFINDER`,
`SET_POSITION_TARGET_LOCAL_NED`, …). **pymavlink** is the Python library that
encodes/decodes them. The connection string `udp:127.0.0.1:14550` is the standard
ArduPilot **SITL** (software-in-the-loop) endpoint; on hardware it's a serial UART at
`MAVLINK_BAUD`.

What `MavlinkInterface` does (mavlink_iface.py lines 33–73):

- **`send_velocity_body(vx,vy,vz)`** — sends `SET_POSITION_TARGET_LOCAL_NED` with
  frame `MAV_FRAME_BODY_NED` and a **type-mask** `0b0000111111000111` that tells
  ArduCopter "ignore position and acceleration fields; use only the three velocity
  fields." This is *the* command the landing controller ultimately produces. Being
  able to read that bitmask (each bit ignores one field) is a useful skill — see the
  resources.
- **`read_velocity_body()`** — receives `LOCAL_POSITION_NED` (velocity in NED) and
  `ATTITUDE` (for yaw), then rotates NED→body via `frames.ned_to_body` (Module 02).
  This is the **ego-motion** that the EKF folds into `predict` (principle 3). Note it
  comes from *ArduCopter's* filter, not a homemade one (principle 4).
- **`read_rc_mode()`** — reads `HEARTBEAT.custom_mode`; `9` is ArduCopter's LAND mode
  number. This is how the pilot's mode switch tells the companion "begin landing."
- **`read_agl()`** — reads `RANGEFINDER.distance` (height above ground from a downward
  rangefinder). AGL = "Above Ground Level."
- **`read_attitude()`** — roll/pitch/yaw in degrees.

Notice `from pymavlink import mavutil` is imported **lazily**, inside `__init__`
(line 37). That's deliberate: it means the *package* imports fine on a laptop with no
pymavlink/autopilot, and you only pay for the hardware dependency when you actually
construct the real interface. Same trick for `pyserial` in the gimbal.

## The real/mock `Protocol` pattern (learn this)

Look at `VehicleBackend` (mavlink_iface.py lines 24–30):

```python
@runtime_checkable
class VehicleBackend(Protocol):
    def send_velocity_body(self, vx, vy, vz) -> None: ...
    def read_velocity_body(self) -> np.ndarray: ...
    def read_rc_mode(self) -> str: ...
    def read_agl(self) -> float: ...
    def read_attitude(self) -> tuple: ...
```

A `Protocol` is Python's **structural interface** ("duck typing, but checked"): any
class with these methods *is* a `VehicleBackend` — no inheritance required. Two
implementations satisfy it:

- `MavlinkInterface` — talks to real ArduCopter/SITL.
- `MockMavlink` — a pure-software backend that records the last setpoint and serves
  injected `agl`/`rc`/`attitude`/`drone_vel` for tests and the `--sim` loop.

The FSM (Module 07) depends only on the **protocol**, so swapping real ⇄ mock is a
one-line change at construction. This is **dependency injection**, and it's the
reason the entire perception→control→actuation chain is testable on a laptop with a
synthetic camera. The same pattern appears for:

- the gimbal — `GimbalBackend` / `SimpleBGCInterface` / `MockGimbal`
  (gimbal_iface.py),
- the current sensor — `CurrentSensor` / `MockCurrentSensor` (in `exp_bec_current.py`,
  Module 08), where the real backend would wrap an INA219/INA226 shunt monitor.

Recognizing this pattern across the codebase is a big part of "understanding the
architecture." When you see a `Protocol` + a `Mock*` + a `*Interface`, you're
looking at a **hardware seam**: the line where software meets a physical device,
designed so the software above it never knows or cares which side is connected.

## The gimbal interface

`gimbal_iface.py` mirrors the MAVLink one. `SimpleBGCInterface` would encode SimpleBGC
serial command frames (the actual byte-packing is stubbed as hardware-specific);
`MockGimbal` models the gimbal as a **first-order + slew-limited** actuator advanced
by `integrate(dt)` (lines 61–66) so laptop runs behave like a real, laggy gimbal.
The **SimpleBGC32** is a popular open gimbal controller; the camera is mounted on it
so the bore can be pointed independently of the airframe.

## External resources

- **MAVLink protocol + message dictionary** (look up the messages above):
  https://mavlink.io/en/ — see `SET_POSITION_TARGET_LOCAL_NED`,
  `LOCAL_POSITION_NED`, `HEARTBEAT`, `ATTITUDE`, `RANGEFINDER`. The type-mask is
  documented on the setpoint message page.
- **pymavlink** source + examples: https://github.com/ArduPilot/pymavlink and the
  "Using pymavlink" dev docs: https://www.ardusub.com/developers/pymavlink.html
- **ArduPilot dev docs** — companion computers, guided-mode commands, and
  **SITL** (so you can run a *real* autopilot in software, no drone):
  https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html and
  https://ardupilot.org/dev/docs/companion-computers.html
- **ArduPilot source** (to see the *other* side of every message this code sends):
  https://github.com/ArduPilot/ardupilot — and ArduCopter's existing precision-landing
  feature for context: https://ardupilot.org/copter/docs/precision-landing-with-irlock.html
- **DroneKit-Python / MAVSDK** — higher-level companion APIs, useful comparison to the
  raw-pymavlink approach this repo takes: https://github.com/dronekit/dronekit-python ,
  https://mavsdk.mavlink.io/
- **Python `Protocol` / structural typing** (the pattern): PEP 544
  (https://peps.python.org/pep-0544/) and the `typing.Protocol` docs.
- **Tiziano Fiorenzani's precision-landing repo** again — to see a real
  pymavlink/dronekit companion sending landing commands:
  https://github.com/tizianofiorenzani/how_do_drones_work

## Exercises

1. **Decode the type-mask.** `0b0000111111000111` in `send_velocity_body`. Using the
   MAVLink `SET_POSITION_TARGET_LOCAL_NED` field-bit table, list which fields are
   *ignored* (1) and which are *used* (0). Confirm only the three velocity fields are
   active.
2. **Trace ego-motion.** Follow a velocity number from ArduCopter to the EKF: which
   message carries it, which frame it arrives in, where it's rotated, and which EKF
   line consumes it. Write the 4-step path.
3. **Use the seam.** In a REPL, build a `MockMavlink(drone_vel=(1,0,0))`,
   call `read_velocity_body()`, and confirm it returns `[1,0,0]`. Then set
   `attitude` and re-derive what the *real* interface would return for the same NED
   velocity at yaw 90°. (Pencil + `frames.ned_to_body`.)
4. **(Optional, ambitious) Stand up SITL.** Follow the ArduPilot SITL docs to run a
   simulated ArduCopter, point `MAVLINK_CONN` at it, and have `MavlinkInterface`
   `wait_heartbeat()` succeed. This bridges "laptop sim" and "real autopilot" without
   any drone.
