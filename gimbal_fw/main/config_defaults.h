// Place-holder values until Pasha and Aga give them.
#pragma once

// --- Joint limits (rad), protect cables + go in IK clamp ---
// DRAFT (plan §T1 example) -- TODO(Pasha): confirm mechanical limits
#define JOINT_PITCH_MIN_RAD   (-1.9199f)   // -110 deg
#define JOINT_PITCH_MAX_RAD   ( 0.3491f)   //  +20 deg
#define JOINT_ROLL_MIN_RAD    (-0.6981f)   //  -40 deg
#define JOINT_ROLL_MAX_RAD    ( 0.6981f)   //  +40 deg
#define JOINT_YAW_MIN_RAD     (-2.9671f)   // -170 deg
#define JOINT_YAW_MAX_RAD     ( 2.9671f)   // +170 deg

// --- Kinematic chain (§2.3) ---
// TODO(Pasha): confirm mechanical chain is yaw -> roll -> pitch; tell before changing IK.

// --- Open-loop drive ("FOC"), §4.3 ---
#define FOC_PHASE_VOLTAGE_V       1.0f     // u: start low -- TODO(measure, T8): raise until it holds, stay <~0.5A
#define FOC_RAMP_TIME_S           0.5f     // §4.6: ease rotor in over 0.5s on enable
#define FOC_PHASE_RESISTANCE_OHM  0.0f     // TODO(measure, T0.3): half of phase-to-phase resistance

// --- Bench power / safety (§2.6) ---
#define BENCH_PSU_CURRENT_LIMIT_A  0.5f    // 0.5-1 A during bench work; never raw LiPo until T10 done

// --- Control law (stab_step), §4.2 ---
#define STAB_GAIN_K            1.0f        // TODO(tune, T11/T12): start ~5 Hz equivalent, raise until it rings, halve
#define STAB_OMEGA_MAX_RAD_S   3.1416f     // TODO(tune, T12): max joint slew rate

// --- Mahony attitude filter, §T5 ---
#define MAHONY_KP   1.0f    // TODO(tune, T5): starting gain
#define MAHONY_KI   0.02f   // TODO(tune, T5): starting gain
#define MAHONY_ACCEL_GATE_G  0.15f  // skip accel correction when │|a|-1g│ > this

// --- MPU6050 config, §T4 (fixed by plan, not pending) ---
#define MPU_GYRO_RANGE_DPS    1000
#define MPU_ACCEL_RANGE_G     4
#define MPU_DLPF_CONFIG       2     // ~94 Hz BW, ~3ms delay; required for 1kHz output

// --- MCPWM carrier, §4.3/T8 (verify on scope) ---
#define MCPWM_RESOLUTION_HZ    20000000   // 20 MHz tick
#define MCPWM_PERIOD_TICKS     800        // target 25 kHz carrier -- TODO(verify on scope)

// --- Protocol / comms timing, §2.4 (fixed by plan) ---
#define TELEM_RATE_HZ          50
#define SET_TARGET_RATE_HZ_MIN 20
#define SET_TARGET_RATE_HZ_MAX 50
#define COMMS_WATCHDOG_MS      200   // no valid frame -> NADIR hold

// --- Loop timing, §2.4 (fixed by plan) ---
#define IMU_SAMPLE_RATE_HZ     1000
#define CTRL_LOOP_RATE_HZ      1000
#define CTRL_LOOP_BUDGET_US    400
