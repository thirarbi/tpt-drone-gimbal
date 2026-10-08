# landing_sw — Sequence Diagram

The vision precision-landing companion runs one closed loop per camera frame
(~30 fps). `LandingFSM.step()` is the orchestrator: it predicts, perceives,
filters, decides a state, then acts. The diagrams below trace that flow.

## 1. Per-frame closed loop (`LandingFSM.step`)

This is the hot path — one pass per frame, driven by `main.run_sim` (sim) or the
hardware capture loop (real).

```mermaid
sequenceDiagram
    autonumber
    participant Loop as main.run_sim / capture loop
    participant Cam as Camera<br/>(synthetic_markers / picamera2)
    participant FSM as LandingFSM
    participant MAV as Mavlink (Mock/real)
    participant EKF as RelativeMarkerEKF
    participant Det as MarkerDetector
    participant Frm as frames
    participant Gim as Gimbal (Mock/real)
    participant Pt as PointingController
    participant Land as LandingController
    participant DB as TelemetryDB

    Loop->>Cam: render_view(agl, offset, gimbal angles)
    Cam-->>Loop: gray frame (DETECT_RES)
    Loop->>FSM: step(frame, agl, rc, attitude, camera_ok)

    Note over FSM,EKF: predict — ego-motion folded in (principle 3)
    FSM->>MAV: read_velocity_body()
    MAV-->>FSM: drone_vel_body
    FSM->>EKF: predict(dt, drone_vel_body)
    Note right of EKF: x[:3] -= drone_vel_body·dt

    Note over FSM,Frm: perceive
    FSM->>Det: detect(frame, est_agl_m=agl)
    Det->>Det: ArUco detect + solvePnP(IPPE_SQUARE)<br/>reproj gate (REPROJ_ERR_MAX_PX)
    Det-->>FSM: Pose (camera frame, valid?)

    alt pose.valid
        FSM->>Gim: get_angles()
        Gim-->>FSM: pitch, roll, yaw
        FSM->>Frm: camera_point_to_body(pose.t, angles)
        Frm-->>FSM: pos_body (FRD)
        FSM->>EKF: update(pos_body)
        Note right of EKF: chi² gate (EKF_GATE_CHI2)
        EKF-->>FSM: accepted (bool)
    end

    Note over FSM: decide — _transition_landing(agl)
    FSM->>FSM: pick State<br/>(ACTIVE_LANDING / COASTING /<br/>GIMBAL_SEARCH / ABORT / NAVIGATE)

    Note over FSM,Land: act — _act(agl, veh_yaw)
    alt ACTIVE_LANDING / COASTING / TRANSITION
        FSM->>Pt: step(dt, ekf.position)
        FSM->>Gim: set_angles(LAND_PITCH, 0, 0)  %% locked nadir
        FSM->>Land: send(ekf.position, ekf.velocity, agl)
        Land->>Land: v = Kp·err + Gff·v_target (principle 2)
        Land->>MAV: send_velocity_body(vx, vy, vz)
    else GIMBAL_SEARCH
        FSM->>Gim: set_angles(LAND_PITCH, 0, sweep_yaw)
        FSM->>MAV: send_velocity_body(0,0,0)  %% hold
    else ABORT
        FSM->>Land: abort()  %% climb at ABORT_CLIMB_RATE
        Land->>MAV: send_velocity_body(0,0,-climb)
    end

    FSM->>DB: log_tick(...)
    FSM-->>Loop: state
    Loop->>Gim: integrate(dt)  %% sim only: slew lag
    Loop->>Loop: integrate drone kinematics from last_setpoint
```

## 2. Marker-loss transition policy (`_transition_landing`)

How the FSM reacts when the EKF stops accepting updates (principles 3 & 4).

```mermaid
sequenceDiagram
    autonumber
    participant FSM as LandingFSM
    participant EKF as RelativeMarkerEKF

    Note over FSM: _since_seen accumulates while marker is lost
    alt _since_seen < MARKER_LOSS_COAST_S (2.0 s)
        Note over FSM,EKF: COASTING — fly the constant-velocity prediction<br/>(companion bridges 30 fps gaps; EKF3 dead-reckons vehicle)
    else loss ≥ coast window, and MIN_AGL ≤ agl ≤ MAX_AGL
        Note over FSM: GIMBAL_SEARCH — altitude-gated yaw sweep
        FSM->>FSM: _search_elapsed += dt
        alt reacquired
            FSM->>EKF: reset()  %% drop stale velocity
            Note over FSM: → ACTIVE_LANDING
        else _search_elapsed > SEARCH_TIMEOUT_S (8 s)
            Note over FSM: → ABORT
        end
    else loss at unsafe AGL (below MIN or above MAX)
        Note over FSM: → ABORT
    end
```

## 3. Startup wiring (`main.run_sim`)

One-time dependency injection before the loop.

```mermaid
sequenceDiagram
    autonumber
    participant Main as run_sim
    participant DB as TelemetryDB
    participant FSM as LandingFSM

    Main->>Main: build MarkerDetector, RelativeMarkerEKF,<br/>MockMavlink, MockGimbal,<br/>PointingController, LandingController
    Main->>DB: start_flight("sim_descent")
    DB-->>Main: flight_id
    Main->>FSM: LandingFSM(detector, ekf, pointing, landing,<br/>gimbal, mav, db, flight_id)
    loop until flare or timeout
        Main->>FSM: step(...)
    end
    Main->>DB: commit(); close()
```

> **Real vs. sim** — the only swap between hardware and `--sim` is at construction:
> `MockMavlink`/`MockGimbal`/`synthetic_markers` vs.
> `MavlinkInterface`/`SimpleBGCInterface`/`picamera2`. Every consumer takes a plain
> `HxW uint8` frame and the same interface methods, so `LandingFSM.step` is identical
> on both paths.
