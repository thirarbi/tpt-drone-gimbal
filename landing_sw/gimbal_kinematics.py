import numpy as np

YAW, ROLL, PITCH = 0, 1, 2 #Index order of Q
SINGULAR_SIN = 0.995 # |sin(roll)|

#PLACEHOLDERS until Pasha's
LIMITS = np.radians([(-170, 170), (-40, 40), (-110, 20)])
#=========================================================

def _rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

def _ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

def _rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

def fk(q):
    return _rz(q[YAW]) @ _rx(q[ROLL]) @ _ry(q[PITCH])

"""Returns (q, near_singular)"""
def ik(R):
    s_roll = np.clip(R[2,1], -1.0, 1.0)
    q = np.empty(3)
    q[ROLL] = np.arcsin(s_roll)
    q[YAW] = np.arctan2(-R[0,1], R[1,1])
    q[PITCH] = np.arctan2(-R[2,0], R[2,2])
    return q, abs(s_roll) > SINGULAR_SIN

def clamp_limits(q):
    return np.clip(q, LIMITS[:, 0], LIMITS[:, 1])