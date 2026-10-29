// Place-holder values until Pasha and Aga give them.
#pragma once

// --- What the firmware runs at boot (TESTING.md says which one to use when) ---
#define APP_GIMBAL         0   // normal: boot sequence (§4.6), stabilise, talk to the Pi
#define APP_IMU_ONLY       1   // T4/T5: IMU + Mahony stats, servos untouched
#define APP_BUS_SCAN       2   // T8: list every servo that answers (IDs 0-252, 4 bauds)
#define APP_SERVO_SWEEP    3   // T8/T10: each joint swings +-20 deg around zero_tick, then all together
#define APP_JOINT_MONITOR  4   // T16: torque OFF, print ticks/angles + camera attitude while you move joints
#define APP_MPU_SIXPOSE    5   // re-measure accel offsets (MPU-CALIBRATION.md)
#define APP_SERVO_SPEEDUP  6   // ONE-SHOT: Return Delay 0 / SRL 1 / 1 Mbit/s on every servo that answers
#ifndef APP_MODE
#define APP_MODE           APP_GIMBAL
#endif

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
#define DXL_BAUD                 115200    // Aga's servos today -- 1000000 only after APP_SERVO_SPEEDUP
#define SERVO_RATE_HZ            100       // stab_step + bus cycle; 50 if 3 servos stay at 115200
#define SERVO_READ_TIMEOUT_US    5000      // Aga's value; > one read round trip (~2.85 ms @115200) -- TODO(measure, T8)
#define SERVO_SPEED_BOOT         50        // §4.6: slow walk to NADIR
#define SERVO_SPEED_RUN          0         // 0 = max -- TODO(tune, T12); Aga used 300
#define SERVO_BOOT_WALK_MS       3000      // boot speed this long after torque-on, then SERVO_SPEED_RUN
#define SERVO_TORQUE_LIMIT       1023      // 0..1023; lower it for a first power-up (T16)
#define SERVO_WRITE_ANGLE_LIMITS 1         // T10: write CW/CCW Angle Limit (EEPROM) from the joint limits at boot
#define SERVO_QMEAS_STALE_MS     30        // §4.2: fall back to q_cmd when readback is older than this
#define SERVO_VOLTAGE_MIN_V      6.0f      // XL-320 supply range; refuse torque outside it
#define SERVO_VOLTAGE_MAX_V      8.4f

// --- Control law (stab_step), §4.2 ---
#define STAB_MODE              STAB_MODE_IK  // STAB_MODE_PID = fallback if the IK loop fights you
#define STAB_GAIN_K            0.3f        // per 100 Hz tick, ~5.7 Hz -- TODO(tune, T12): raise until it rings, halve
#define STAB_OMEGA_MAX_RAD_S   3.1416f     // TODO(tune, T12): max joint slew rate
#define STAB_PID_KP            20.0f       // 1/s  -- PID fallback only, TODO(tune)
#define STAB_PID_KI            2.0f        // 1/s^2
#define STAB_PID_KD            0.0f        // s^0 (D on error)
#define STAB_PID_I_MAX         0.5f        // rad/s

// --- Mahony attitude filter, §T5 ---
#define MAHONY_KP   1.0f    // TODO(tune, T5): starting gain
#define MAHONY_KI   0.02f   // TODO(tune, T5): starting gain
#define MAHONY_ACCEL_GATE_G  0.15f  // skip accel correction when │|a|-1g│ > this
#define MAHONY_INTEGRAL_MAX_RAD_S  0.05f
#define MAHONY_KP_BOOT          10.0f // Converge fast after boot
#define MAHONY_WARMUP_SAMPLES   1000  // ~1 s at 1 kHz

// --- Boot gyro bias calibration, §4.6 step 2 ---
#define GYRO_CAL_SAMPLES       2000     // 2 s at 1 kHz, gimbal must be still
#define GYRO_CAL_MAX_STD_RADS  0.01f    // ~0.6 deg/s: above this "someone's touching it", retry
#define GYRO_CAL_ATTEMPTS      5        // then fall back to mpu_unit.h


// --- MPU6050 config, §T4 (fixed by plan, not pending) ---
#define MPU_GYRO_RANGE_DPS    1000
#define MPU_ACCEL_RANGE_G     4
#define MPU_DLPF_CONFIG       2     // ~94 Hz BW, ~3ms delay; required for 1kHz output

// --- What clocks ctrl_task (plan Rev 4) ---
// 1 = MPU6050 data-ready IRQ on MPU_INT_GPIO (plan §2.4, needs the INT wire)
// 0 = 1 ms FreeRTOS tick (no INT wire; needs CONFIG_FREERTOS_HZ=1000). Current setting.
#define MPU_USE_INT           0


// --- Protocol / comms timing, §2.4 (fixed by plan) ---
#define PI_UART_BAUD           115200
#define TELEM_RATE_HZ          50
#define SET_TARGET_RATE_HZ_MIN 20
#define SET_TARGET_RATE_HZ_MAX 50
#define COMMS_WATCHDOG_MS      200   // no valid frame -> NADIR hold

// --- Loop timing, §2.4 (fixed by plan) ---
#define IMU_SAMPLE_RATE_HZ     1000
#define CTRL_LOOP_RATE_HZ      1000
#define CTRL_LOOP_BUDGET_US    400

// --- Dev logging over the USB console ---
#define LOG_CSV_HZ     0        // >0: also print "D,..." lines this often for tools/serial_log.py (50 is plenty)
#define MPU_I2C_HZ     400000
