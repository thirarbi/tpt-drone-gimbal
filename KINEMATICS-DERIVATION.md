The motors of the 3-axis gimbal uses **joint-space** ($\theta z, \theta x, \theta y$) which are read from encoders and read form motors. However, the IMU uses **orientation-space** to know the camera's attitude, as a rotation matrix or quaternion. Thus, we'll need to derive the IMU's output into joint-space through inverse-kinematics.

### Output of IMU
IMU gives out raw data.

| **Sensor**            | **Measures**                     | **Units**    |
| --------------------- | -------------------------------- | ------------ |
| Gyroscope             | Angular rate about 3 body axes   | rad/s or °/s |
| Accelerometer         | Specific force (thurst and drag) | $m/s^2$or g  |
| Magnetometer (9-axis) | Magnetic field vector            | $\mu T$      |
The IK function will take rotation matrix `R` for the camera relative to the body.
### Derivation
Starting with the three elementary rotations:
![[three-axis.png]]

![[inverse_kinematics.png]]

**Note:** C's float error can push `1.0000001`, and `asin` of that is NaN. Thus, clamp `R32` into `[-1, 1]`.

In code (note the index shift. start from `[0]`):
```c
// Clamp: asinf(1.0000001f) = NaN
float s_roll = fminf(fmaxf(R->m[2][1], -1.0f), 1.0f);
q[ROLL] = asinf(s_roll);
q[YAW]  = atan2f(-R->m[0][1], R->m[1][1]);
q[PITCH] = atan2f(-R->m[2][0], R->m[2][2]);
```

```
q_yaw, q_roll, q_pitch,  R00, R01, R02, R10, R11, R12, R20, R21, R22
└──── q (3 numbers) ───┘ └────────── R = fk(q) (9 numbers) ──────────┘
```


