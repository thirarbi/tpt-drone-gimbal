// Place-holder values until Pasha and Aga give them.
#pragma once

// --- Joint limits (rad), protect cables + go in IK clamp ---
// DRAFT (plan §T1 example) -- TODO(Pasha): confirm mechanical limits
#define JOINT_PITCH_MIN_RAD   (-1.9199f)   // -110 deg
#define JOINT_PITCH_MAX_RAD   ( 0.3491f)   //  +20 deg
#define JOINT_ROLL_MIN_RAD    (-0.6981f)   //  -40 deg
#define JOINT_ROLL_MAX_RAD    ( 0.6981f)   //  +40 deg
#define JOINT_YAW_MIN_RAD     (-2.5307f)   // -145 deg: XL-320 has only 300 deg of travel
#define JOINT_YAW_MAX_RAD     ( 2.5307f)   // +145 deg

// --- Kinematic chain (§2.3) ---
// TODO(Pasha): confirm mechanical chain is yaw -> roll -> pitch; tell before changing IK.

// --- XL-320 servo bus, §2.4/§4.3 ---
#define DXL_BAUD                 115200    // Aga's servos today -- TODO(T8): 1000000 after reconfig
#define SERVO_RATE_HZ            100       // stab_step + bus cycle; 50 if stuck at 115200
#define SERVO_READ_TIMEOUT_US    5000      // Aga's value; > one read round trip (~2.85 ms @115200) -- TODO(measure, T8)
#define SERVO_SPEED_BOOT         50        // §4.6: slow walk to NADIR
#define SERVO_SPEED_RUN          0         // 0 = max -- TODO(tune, T12); Aga used 300
#define SERVO_QMEAS_STALE_MS     30        // §4.2: fall back to q_cmd when readback is older than this
#define SERVO_VOLTAGE_MIN_V      6.0f      // XL-320 supply range; refuse torque outside it
#define SERVO_VOLTAGE_MAX_V      8.4f

// --- Control law (stab_step), §4.2 ---
#define STAB_GAIN_K            1.0f        // TODO(tune, T11/T12): per 100 Hz tick; start ~5 Hz equivalent, raise until it rings, halve
#define STAB_OMEGA_MAX_RAD_S   3.1416f     // TODO(tune, T12): max joint slew rate

// --- Mahony attitude filter, §T5 ---
#define MAHONY_KP   1.0f    // TODO(tune, T5): starting gain
#define MAHONY_KI   0.02f   // TODO(tune, T5): starting gain
#define MAHONY_ACCEL_GATE_G  0.15f  // skip accel correction when │|a|-1g│ > this
#define MAHONY_INTEGRAL_MAX_RAD_S  0.05f
#define MAHONY_KP_BOOT          10.0f // Converge fast after boot
#define MAHONY_WARMUP_SAMPLES   1000  // ~1 s at 1 kHz


// --- MPU6050 config, §T4 (fixed by plan, not pending) ---
#define MPU_GYRO_RANGE_DPS    1000
#define MPU_ACCEL_RANGE_G     4
#define MPU_DLPF_CONFIG       2     // ~94 Hz BW, ~3ms delay; required for 1kHz output

// --- What clocks ctrl_task (plan Rev 4) ---
// 1 = MPU6050 data-ready IRQ on MPU_INT_GPIO (plan §2.4, needs the INT wire)
// 0 = 1 ms FreeRTOS tick (no INT wire; needs CONFIG_FREERTOS_HZ=1000). Current setting.
#define MPU_USE_INT           0

// --- MPU6050 per-unit calibration, see MPU-CALIBRATION.md ---
#define MPU_CALIB_SIXPOSE     0     // 1 = run the ~45 s six-pose accel test at boot (blocks start-up); 0 = normal boot

// --- Protocol / comms timing, §2.4 (fixed by plan) ---
#define TELEM_RATE_HZ          50
#define SET_TARGET_RATE_HZ_MIN 20
#define SET_TARGET_RATE_HZ_MAX 50
#define COMMS_WATCHDOG_MS      200   // no valid frame -> NADIR hold

// --- Loop timing, §2.4 (fixed by plan) ---
#define IMU_SAMPLE_RATE_HZ     1000
#define CTRL_LOOP_RATE_HZ      1000
#define CTRL_LOOP_BUDGET_US    400

// --- T4 dev logger, DECISIONS §5 ---
#define T4_STREAM      0        // 0 = measure (stats text once a second), 1 = record (binary frames)
#define T4_QUEUE_LEN   1024     // ~1 s of frames
#define MPU_I2C_HZ     400000
