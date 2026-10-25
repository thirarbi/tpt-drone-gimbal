[Gimbal Lock](https://www.compu-tools.com/blog/2026-03-07-gimbal-lock/) is a singularity phenomenon that occurs when representing 3D rotations using Euler angles.

When two rotation axis align at a specific angle (typically Pitch $\pm90\degree$), one degree of freedom is lost, and any controller attempting to compensate may command joints to rotate hundreds of degrees with catastrophic results. The inverse kinematics algorithm could be mathematically correct, and run fine in simulation but fails.

A gimbal are rings rotating around the X, Y, and Z axes respectively can maintain the orientation of an inner device from any direction.

![[Pasted image 20261025210505.png]]

_The physical gimbal structure: when the Y-axis ring rotates 90°, the X and Z axes become parallel, losing one degree of freedom_

The problem occurs the moment the middle ring, typically the Y-axis or Pitch axis) reaches $\pm90\degree$. At that point, the outermost ring's rotation axis and the innermost ring's rotation axis become parallel. Physically, both rings can only rotate in essentially the same direction and having three rings, the number of independently representable rotation directions drop to two.

This is **Gimbal Lock**. The number of rings hasn't changed; two of them have simply started doing the same thing.
### Why Euler Angles Have Singularities
3D rotation is mathematically defined as an operation on the Lie Group called **SO(3) (Special Orthogonal Group)** the set of 3x3 orthogonal matrices with determinant 1, encompassing all 3D rotations that preserve length and orientation. Euler angles are an attempt to map this space to three real-number coordinates.

Now, the topological structure of SO(3) is such that no globally smooth coordinate system (a mapping free of coordinate singularities) exist. This is analogous to the impossibility of perfectly representing a sphere ($S^2$) as a flat map. As a consequence of the [**Hairy Ball Theorem**](https://en.wikipedia.org/wiki/Hairy_ball_theorem) (sometimes called the hedgehog theorem), no globally continuous unit vector field exists on SO(3). Since Euler angles are merely a coordinate parameterization, they cannot escape this barrier. At least one singularity must arise. Gimbal Lock is the most prominent example.

![[Pasted image 20261025213654.png|622]]

A commonly used formula for converting quaternion to ZYX Euler angles:
```python
import numpy as np

def quaternion_to_euler_zyx(w, x, y, z):
    # Roll (φ, X-axis rotation)
    roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    # Pitch (θ, Y-axis rotation)
    sin_pitch = 2*(w*y - z*x)
    sin_pitch = np.clip(sin_pitch, -1.0, 1.0)
    pitch = np.arcsin(sin_pitch)
    # Yaw (ψ, Z-axis rotation)
    yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return roll, pitch, yaw
```

As `pitch` approaches `+-90` degrees, both args passed to `arctan2` approach zero. At that point, `atan2(0,0)` is mathematically undefined and accumulated floating-point errors cause Roll and Yaw values to blow up.

### Chosen Avoidance Strategy
There are two different places to deal with Gimbal Lock:
1. **The math.** Representing attitude as three Euler angles has a singularity. This can be removed by using quaternions or rotation matrices.
2. **The mechanism.** Your gimbal has three physical joints, and the joint angles are Euler angles by construction. No representation changes the fact that, at the lock configuration, two motors would turn the same axis.

The Mahony filter keeps attitude as a quaternion and as `R_wc`. Those have no singularity. You only convert to joint angles in `ik` at the end, because the motors need joint angles. Thus the singularity only has to be handled in one function. However, the main strategy must be at the mechanism itself.  A singularity always occurs when the **middle** joint reaches $\pm90\degree$. So the question is which joint to place in the middle.

Since the camera works at pitch = $-90\degree$ (nadir), if the pitch were the middle joint, the *normal wirking point* would sit exactly on the singularity. So the chain is yaw -> roll -> pitch, with **roll** in the middle. The singularity moves to roll $\pm90\degree$, which a stabilised camera never needs.

In `gimbal_kinematics.py`, the `LIMITS` roll to $\pm40\degree$, and `clamp_limits` enforces it. The singularity is $50\degree$ beyond the largest roll the gimbal can be commanded to, which is the real source of safety here. The same limits also protect the cables. Now, we can put 0.9945 flag ($84.3\degree$) in `ik` as a safety net for cases the limits don't cover. So for example if the IMU-derived `R_wb` is noisy, so the IK *input* can ask for a roll the limits would never allow. The flag marks that the yaw/pitch outputs can't be trusted. So when that happens, it'll use $q_{cmd} \ += \text{clamp}(K \cdot err, \pm \omega_{max}\cdot dt)$. This makes it so that if IK returns garbage, the commanded joints can only move a bounded amount per tick, so a bad value becomes a slow wander.