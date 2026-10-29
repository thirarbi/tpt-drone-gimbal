import pathlib
import numpy as np
from mahony_ref import qmul, q_to_R, run

ROOT = pathlib.Path(__file__).resolve().parents[2]
OUT = ROOT / "gimbal_fw" / "host_test" / "golden_mahony.csv"

rng = np.random.default_rng(7)
dt, N = 0.001, 20000

# Rotation by a about x/y/z (axis 0/1/2)
def qax(axis, a):
    q = np.zeros(4); q[0] = np.cos(a / 2); q[1 + axis] = np.sin(a / 2)
    return q

t = np.arange(N + 1) * dt
roll  = np.radians(20) * np.sin(2*np.pi*0.5*t)
pitch = np.radians(15) * np.sin(2*np.pi*0.3*t + 1.0)
yaw   = np.radians(10) * t

qt = np.array([qmul(qmul(qax(2, 
                             yaw[k]), 
                             qax(0, roll[k])), 
                             qax(1, pitch[k])) for k in range(N + 1)])

bias = np.array([0.002, -0.003, 0.001])                 # RESIDUAL bias after calibration

gyro = np.empty((N, 3)); acc = np.empty((N, 3))

for k in range(N):
    qc = qt[k] * np.array([1, -1, -1, -1])

    # true body rate from consecutive attitudes
    gyro[k] = 2 * qmul(qc, qt[k + 1])[1:] / dt

    # specific force in the camera frame
    acc[k] = -q_to_R(qt[k]).T @ np.array([0, 0, 1.0])  

gyro += bias + rng.normal(0, 0.01, gyro.shape)
acc  += rng.normal(0, 0.01, acc.shape)

qe, _ = run(gyro, acc, dt, kp=1.0, ki=0.02)

# Truth check, independent of C: angle between estimated and true "down" (yaw-proof)
err = np.degrees([np.arccos(np.clip((q_to_R(qe[k])[2]) @ (q_to_R(qt[k])[2]), -1, 1)) for k in range(N)])

print(f"tilt error after 3 s: rms {np.sqrt((err[3000:]**2).mean()):.3f} deg, max {err[3000:].max():.3f} deg")

assert err[3000:].max() < 1.0

M = 2000 # First 2 s only: includes the convergence transient
np.savetxt(OUT, 
           np.hstack([gyro[:M], acc[:M], qe[:M]]), 
           delimiter=",", 
           fmt="%.9g",
           header="gx,gy,gz,ax,ay,az,qw,qx,qy,qz")
