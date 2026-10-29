"""
The Mahony Filter
Explanation read MAHONY-FILTER.md in this directory.
Or if it has moved, somewhere inside of root/docs
"""

import numpy as np

def qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1*w2 - x1*x2 - y1*y2 - z1*z2,
                     w1*x2 + x1*w2 + y1*z2 - z1*y2,
                     w1*y2 - x1*z2 + y1*w2 + z1*x2,
                     w1*z2 + x1*y2 - y1*x2 + z1*w2])

def q_to_R(q):
    w, x, y, z = q
    return np.array([[1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)],
                     [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)],
                     [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)]])

def mahony_step(q, integ, gyro, acc, dt, kp, ki, gate_g=0.15, i_max=0.05):
    w, x, y, z = q

    v = -np.array([2*(x*z - w*y), 2*(y*z + w*x), w*w - x*x - y*y + z*z])
    an = np.linalg.norm(acc)
    e = np.zeros(3)

    # g gating
    if an > 1e-6 and abs(an - 1.0) <= gate_g:
        e = np.cross(acc / an, v)
    
    integ = np.clip(integ + ki * e * dt, -i_max, i_max)
    g = gyro + kp * e + integ

    # q_dot = 0.5 q ⊗ (0, ω)
    q = q + 0.5 * dt * qmul(q, np.array([0.0, *g]))

    # Renormalize every step
    return q / np.linalg.norm(q), integ

def run(gyro, acc, dt, kp, ki, gate_g=0.15, i_max=0.05):
    q, integ = np.array([1.0, 0, 0, 0]), np.zeros(3)
    out = np.empty((len(gyro), 4))
    for k in range(len(gyro)):
        q, integ = mahony_step(q, integ, gyro[k], acc[k], dt, kp, ki, gate_g, i_max)
        out[k] = q
    return out, integ