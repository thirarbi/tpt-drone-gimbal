import numpy as np
from landing_sw import frames, gimbal_kinematics as gk

def test_camera_to_body_rot():
    rng = np.random.default_rng(0)
    for a in rng.uniform([-110, -40, -170], [20, 40, 170], size=(200, 3)):
        R = frames.camera_to_body_rotation(a)
        assert np.allclose(R @ R.T, np.eye(3), atol=1e-9)
        assert np.isclose(np.linalg.det(R), 1.0)

# Asserts if handed tuple in get_angles() order, the function correctly turns
# that into FK order and radians
def test_matches_fk_times_mount():
    pitch, roll, yaw =  -60.0, 10.0, 25.0
    q_rad = np.deg2rad([yaw, roll, pitch]) # gk.fk order: yaw, roll, pitch
    expected = gk.fk(q_rad) @ frames.R_LINK_OPTICAL
    assert np.allclose(frames.camera_to_body_rotation((pitch, roll, yaw)), expected)

def test_nadir_directions():
    R = frames.camera_to_body_rotation((-90.0, 0.0, 0.0))
    assert np.allclose(R @ [0, 0, 1], [0, 0, 1])     # bore -> body Down
    assert np.allclose(R @ [1, 0, 0], [0, 1, 0])     # image right -> body Right
    assert np.allclose(R @ [0, -1, 0], [1, 0, 0])    # image top -> body Forward