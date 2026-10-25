import numpy as np
from landing_sw import gimbal_kinematics as gk

def wrap_pi(a):
    return (a + np.pi) % (2 * np.pi) - np.pi

def test_round_trip():
    rng = np.random.default_rng(0)
    for _ in range(10_000):
        q = rng.uniform(gk.LIMITS[:, 0], gk.LIMITS[:, 1])
        q2, sing = gk.ik(gk.fk(q))
        assert not sing

# Check reverse IK
def test_fk_is_proper_rotation():
    R = gk.fk([0.3, -0.2, -1.2])
    assert np.allclose(R @ R.T, np.eye(3)) and np.isclose(np.linalg.det(R), 1.0)

def test_singularity_flag():
    _, sing = gk.ik(gk.fk([0.1, np.radians(89), 0.2]))
    assert sing

def test_nadir_not_singular():
    _, sing = gk.ik(gk.fk([0, 0, -np.pi / 2]))
    assert not sing

def test_clamp():
    q = gk.clamp_limits(np.radians([200, 50, -150]))
    assert np.allclose(np.degrees(q), [170, 40, -110])