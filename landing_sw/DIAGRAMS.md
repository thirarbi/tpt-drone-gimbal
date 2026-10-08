# landing_sw — Diagram Perancangan

Kumpulan diagram Mermaid untuk perangkat lunak companion *vision precision-landing*
(Raspberry Pi 5 + ArduCam OV9281 + SimpleBGC32 + SpeedyBee F405 / ArduCopter).

---

## 1. Deployment Diagram

```mermaid
flowchart TB
    subgraph PAD["Landing Pad (TF3001 — dapat bergerak)"]
        ARUCO["Nested ArUco<br/>outer id=23 (1.25 m)<br/>inner id=7 (0.10 m)"]
    end

    subgraph DRONE["Wahana (ArduCopter quad)"]
        subgraph GIMBAL["Gimbal SimpleBGC32"]
            CAM["ArduCam OV9281<br/>mono global-shutter<br/>1280x800 @30 fps"]
            BGC["SimpleBGC32 MCU"]
        end

        subgraph PI["Raspberry Pi 5 (Companion)"]
            APP["landing_sw<br/>(proses Python / FSM)"]
            DB[("flightlog.sqlite")]
        end

        FC["SpeedyBee F405<br/>ArduCopter + EKF3"]
        RNG["Rangefinder (AGL)"]
        BEC["BEC 5 V rail"]
    end

    GCS["Ground Control / SITL<br/>udp:127.0.0.1:14550"]

    CAM -- "MIPI CSI-2 (lo-res Y + hi-res)" --> PI
    BGC -- "UART /dev/serial0 @115200<br/>SimpleBGC CMD_CONTROL / realtime" --> PI
    PI  -- "UART @921600 — MAVLink (pymavlink)" --> FC
    FC  -- "SET_POSITION_TARGET / LOCAL_POSITION_NED / ATTITUDE" --> PI
    RNG -- "RANGEFINDER" --> FC
    BEC -- "5 V" --> PI
    BEC -- "5 V" --> BGC
    FC  <-- "MAVLink (udp, SITL/telemetry)" --> GCS
    CAM -. "mengamati" .-> ARUCO
    APP --- DB
```

---

## 2. Software Architecture

Tampilan komponen + aliran data utama. Panah berlabel dengan *apa* yang mengalir;
`LandingFSM` (Modul 1) adalah orkestrator yang membaca estimasi EKF lalu
meneruskannya ke pointing & landing.

```mermaid
flowchart TB
    MAIN["main.py — entry (--sim / hardware)"]
    CFG["config.py — single source of truth (+ Pose)"]

    subgraph PERC["Persepsi (Modul 2)"]
        MK["markers.py · MarkerDetector"]
        FR["frames.py · cam→body→NED"]
    end
    EKF["ekf.py · RelativeMarkerEKF — Modul 3"]
    PT["pointing.py · PointingController — Modul 4"]
    LN["landing.py · LandingController — Modul 5"]
    FSM["fsm.py · LandingFSM — Modul 1 (orkestrator)"]

    subgraph IO["Lapisan I/O"]
        GI["gimbal_iface · SimpleBGC / MockGimbal"]
        MI["mavlink_iface · Mavlink / MockMavlink"]
    end
    DB["telemetry_db · TelemetryDB"]

    subgraph MODEL["Model & Eksperimen"]
        SM["synthetic_markers.py"]
        SH["sitl_harness.py"]
        LB["latency_bench.py"]
    end

    MAIN --> FSM
    MK -- "Pose (kamera)" --> FSM
    FSM -- "pose.t + sudut gimbal live" --> FR
    FR -- "pos_body (FRD)" --> FSM
    FSM -- "predict(ego-motion) / update(pos_body)" --> EKF
    EKF -- "position, velocity (estimasi)" --> FSM
    FSM -- "position" --> PT
    PT -- "sudut bore (telemetri / sweep)" --> FSM
    FSM -- "position, velocity, agl" --> LN
    LN -- "setpoint kecepatan body" --> MI
    FSM -- "set_angles / get_angles" --> GI
    FSM -- "read_velocity_body / agl / rc / attitude" --> MI
    FSM -- "log_tick / log_event" --> DB
    MK -- "kamus ArUco + render" --> SM
    MI -- "ned_to_body" --> FR
    SH --> FSM
    LB --> FSM
    CFG -.-> PERC & EKF & PT & LN & FSM & IO & DB & MODEL & MAIN
```

---

## 3. Finite-State-Machine Diagram

`ACTIVE_LANDING` dan `COASTING` digabung dalam state majemuk **LANDING**
(COASTING hanya jeda kehilangan marker singkat di dalam pendaratan). Dua
transisi bersifat **global** ke semua state dan ditarik dari catatan, bukan
sebagai panah per-state, agar diagram tetap terbaca.

```mermaid
stateDiagram-v2
    [*] --> IDLE

    IDLE --> NAVIGATE : rc≠LAND & agl≥1 m
    NAVIGATE --> IDLE : rc≠LAND & agl<1 m
    IDLE --> TRANSITION : rc=LAND
    NAVIGATE --> TRANSITION : rc=LAND
    TRANSITION --> LANDING : ekf.initialized & marker terlihat

    state LANDING {
        [*] --> ACTIVE_LANDING
        ACTIVE_LANDING --> COASTING : marker hilang <2.0 s
        COASTING --> ACTIVE_LANDING : marker kembali
    }

    LANDING --> GIMBAL_SEARCH : hilang ≥2.0 s & 2.5–30 m
    LANDING --> ABORT : hilang ≥2.0 s di AGL tak aman
    GIMBAL_SEARCH --> LANDING : reakuisisi (ekf.reset)
    GIMBAL_SEARCH --> ABORT : search timeout >8 s

    ABORT --> LANDING : marker kembali (rc tetap LAND)
    ABORT --> GIMBAL_SEARCH : hilang ≥2.0 s & 2.5–30 m

    note left of ABORT
        Memanjat @1.5 m/s. Bukan state final:
        selama rc=LAND ia pulih bila marker
        ditemukan lagi.
    end note
    note right of LANDING
        COASTING = terbang pada prediksi EKF.
        GIMBAL_SEARCH = sapuan yaw bergerbang
        altitude, tahan posisi.
        BERLAKU DI SEMUA STATE:
        kamera gagal → ABORT;
        rc≠LAND → IDLE (agl<1 m) / NAVIGATE.
    end note
```

---

## 4. Sequence Diagram — ACTIVE_LANDING

Satu iterasi `LandingFSM.step()` saat berada di state `ACTIVE_LANDING`
(marker terlihat & diterima EKF).

```mermaid
sequenceDiagram
    autonumber
    participant Host as run_sim / capture loop
    participant FSM as LandingFSM
    participant MAV as Mavlink
    participant EKF as RelativeMarkerEKF
    participant Det as MarkerDetector
    participant Frm as frames
    participant Gim as Gimbal
    participant Pt as PointingController
    participant Land as LandingController
    participant DB as TelemetryDB

    Host->>FSM: step(frame, agl, rc=LAND, attitude)

    Note over FSM,EKF: predict — ego-motion (prinsip 3)
    FSM->>MAV: read_velocity_body()
    MAV-->>FSM: drone_vel_body
    FSM->>EKF: predict(dt, drone_vel_body)
    Note right of EKF: x[:3] -= drone_vel_body·dt

    Note over FSM,Det: perceive
    FSM->>Det: detect(frame, est_agl_m=agl)
    Det->>Det: ArUco + solvePnP(IPPE_SQUARE)<br/>gate reproj ≤ 3 px
    Det-->>FSM: Pose(valid=True)

    FSM->>Gim: get_angles()
    Gim-->>FSM: pitch, roll, yaw
    FSM->>Frm: camera_point_to_body(pose.t, angles)
    Frm-->>FSM: pos_body (FRD)
    FSM->>EKF: update(pos_body)
    Note right of EKF: chi² gate (16.27)
    EKF-->>FSM: accepted=True → _since_seen=0

    Note over FSM: decide → ACTIVE_LANDING

    Note over FSM,Land: act
    FSM->>Pt: step(dt, ekf.position) [telemetri]
    FSM->>Gim: set_angles(-90, 0, 0) [bore nadir terkunci]
    FSM->>Land: send(ekf.position, ekf.velocity, agl)
    Land->>Land: v = Kp·err + Gff·v_target (prinsip 2)
    Land->>MAV: send_velocity_body(vx, vy, vz)

    FSM->>DB: log_tick(...)
    FSM-->>Host: State.ACTIVE_LANDING
```

---

## 5. Class Diagram

```mermaid
classDiagram
    class State {
        <<enum>>
        IDLE
        NAVIGATE
        TRANSITION
        ACTIVE_LANDING
        COASTING
        GIMBAL_SEARCH
        ABORT
    }

    class Pose {
        +ndarray t
        +ndarray rvec
        +str frame
        +bool valid
        +int marker_id
    }

    class LandingFSM {
        +State state
        +float dt
        +step(frame, agl, rc, attitude, camera_ok) State
        -_transition_landing(agl)
        -_act(agl, veh_yaw)
        -_log_tick(agl, pose, accepted)
    }

    class MarkerDetector {
        +float last_reproj_err
        +detect(gray, est_agl_m) Pose
        +detect_with_ambiguity(gray)
    }

    class RelativeMarkerEKF {
        +ndarray x
        +ndarray P
        +bool initialized
        +float last_innov_chi2
        +predict(dt, drone_vel_body)
        +update(z_pos_body) bool
        +reset()
        +position() ndarray
        +velocity() ndarray
    }

    class PointingController {
        +float pitch
        +float roll
        +float yaw
        +step(dt, ekf_position) tuple
        +desired_angles_from_marker(pos_body) tuple
    }

    class LandingController {
        +mav
        +compute_velocity_setpoint(pos, vel, agl) tuple
        +send(pos, vel, agl)
        +abort()
    }

    class GimbalBackend {
        <<interface>>
        +set_angles(pitch, roll, yaw)
        +get_angles() tuple
    }
    class SimpleBGCInterface
    class MockGimbal {
        +integrate(dt)
    }

    class VehicleBackend {
        <<interface>>
        +send_velocity_body(vx, vy, vz)
        +read_velocity_body() ndarray
        +read_rc_mode() str
        +read_agl() float
        +read_attitude() tuple
    }
    class MavlinkInterface
    class MockMavlink {
        +set_agl(agl)
        +set_drone_vel(v)
    }

    class TelemetryDB {
        +start_flight(scenario) int
        +log_tick(...)
        +log_event(flight_id, t, kind, detail)
        +close()
    }

    LandingFSM --> MarkerDetector
    LandingFSM --> RelativeMarkerEKF
    LandingFSM --> PointingController
    LandingFSM --> LandingController
    LandingFSM --> GimbalBackend
    LandingFSM --> VehicleBackend
    LandingFSM --> TelemetryDB
    LandingFSM ..> State
    MarkerDetector ..> Pose : produces
    LandingController --> VehicleBackend
    GimbalBackend <|.. SimpleBGCInterface
    GimbalBackend <|.. MockGimbal
    VehicleBackend <|.. MavlinkInterface
    VehicleBackend <|.. MockMavlink
```

---

## 6. Entity Relationship Diagram — Telemetry Data

```mermaid
erDiagram
    flight ||--o{ tick : "berisi"
    flight ||--o{ event : "mencatat"

    flight {
        int flight_id PK
        real started_at
        text scenario
        text gps
        text notes
    }

    tick {
        int tick_id PK
        int flight_id FK
        real t
        text state
        real marker_id
        real pnp_x
        real pnp_y
        real pnp_z
        real reproj_err
        real meas_x
        real meas_y
        real meas_z
        real meas_accepted
        real innov_chi2
        real ekf_px
        real ekf_py
        real ekf_pz
        real ekf_vx
        real ekf_vy
        real ekf_vz
        real g_pitch
        real g_roll
        real g_yaw
        real sp_vx
        real sp_vy
        real sp_vz
        real agl
        real loop_ms
    }

    event {
        int event_id PK
        int flight_id FK
        real t
        text kind
        text detail
    }
```

---

## 7. Berkas Perancangan Rinci

| Berkas | Peran | Ringkasan |
|---|---|---|
| `config.py` | Sumber tunggal | seluruh parameter; nilai bertanda `# MEASURE` diisi dari eksperimen; mendefinisikan `Pose` |
| `markers.py` | Modul 2 | deteksi ArUco bersarang + PnP `IPPE_SQUARE` + gerbang reproyeksi (≤3 px) |
| `frames.py` | Transform | kamera→body→NED memakai sudut gimbal yang di-stream |
| `ekf.py` | Modul 3 | EKF 6-state; ego-motion sebagai input → kecepatan absolut target |
| `pointing.py` | Modul 4 | estimasi EKF → sudut gimbal (atan2), batas laju slew |
| `landing.py` | Modul 5 | setpoint kecepatan MAVLink + feedforward (`v = Kp·err + Gff·v_target`) |
| `fsm.py` | Modul 1 | mesin 7-state; gating altitude; reset EKF saat reakuisisi |
| `gimbal_iface.py` | I/O | SimpleBGC32 (set+stream sudut) + `MockGimbal` (slew lag) |
| `mavlink_iface.py` | I/O | pymavlink (setpoint, RC, AGL, kecepatan body) + `MockMavlink` |
| `telemetry_db.py` | Data | SQLite skema `flight`/`tick`/`event` |
| `synthetic_markers.py` | Model | render ArUco → validasi persepsi vs ground truth |
| `sitl_harness.py` | Model | loop kendali tertutup; demonstrasi manfaat feedforward |
| `latency_bench.py` | Eksperimen | instrumentasi latensi per-tahap & persentil end-to-end |
| `main.py` | Entry point | picamera2 + pymavlink + SimpleBGC ke FSM; flag `--sim` memakai mock |
