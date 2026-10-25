import sys, pathlib
import numpy as np

# 
ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from landing_sw import gimbal_kinematics as gk

OUT = ROOT / "gimbal_fw" / "host_test" / "golden_kinematics.csv"
rng = np.random.default_rng(42) # Fixed seed, angle number!

with open(OUT, "w") as f:
    f.write("# q_yaw,q_roll,q_pitch,R00,R01,R02,R10,R11,R12,R20,R21,R22\n")
    for _ in range(200):
        q = rng.uniform(gk.LIMITS[:, 0],gk.LIMITS[:, 1])
        row = list(q) + list(gk.fk(q).flatten())
        f.write(",".join(f"{x:.9g}" for x in row) + "\n")