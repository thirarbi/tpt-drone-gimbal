# IMU Mahony Filter Explanation

Sources: [itohi.com — Sensor Fusion: Mahony](https://itohi.com/snippets/filters/sensor-fusion-mahony/) · [Medium — IMU Mahony Filter Explanation](https://medium.com/@k66115704/imu-mahony-filter-explanation-1ae75bf033ab)

The Mahony filter is an orientation estimation algorithm that uses a **complementary filter** approach with PI (proportional–integral) feedback to fuse:

- Gyroscope (angular velocity)
- Accelerometer (gravity direction)
- Magnetometer (optional; **not used in this project**, so yaw is unobservable)

![[Pasted image 20261029190755.png]]

---

## Core Concept

### Complementary Filter

High-pass the gyroscope (smooth, but drifts when integrated) and low-pass the accelerometer/magnetometer (no drift, but noisy):

$$
\text{Orientation} = \alpha \cdot \text{Gyro} + (1-\alpha)\cdot \text{Accel/Mag}
$$

In discrete form, for a single angle $\theta$:

$$
\theta_k = \alpha\,(\theta_{k-1} + \omega_k \Delta t) + (1-\alpha)\,\theta_{\text{acc},k},
\qquad \alpha = \frac{\tau}{\tau + \Delta t}
$$

where $\tau$ is the crossover time constant. Mahony's proportional gain plays the same role: $\tau \approx 1/K_p$.

### PI Controller

**Proportional term**: immediate correction. **Integral term**: compensates the gyroscope bias over time.

$$
\boldsymbol{\omega}_{corrected} = \boldsymbol{\omega}_{gyro} + K_p\,\mathbf{e} + K_i \int \mathbf{e}\,dt
$$

---

## Algorithm Steps

### Step 1: Calculate Error

Take the cross product between the **measured** and the **estimated** direction of a reference vector (order matters, see the note below):

**Accelerometer error:**

$$
\mathbf{e}_a = \mathbf{a}_{measured} \times \hat{\mathbf{g}}^b
$$

where $\hat{\mathbf{g}}^b$ is the estimated gravity direction in the body frame.

**Magnetometer error:**

$$
\mathbf{e}_m = \mathbf{m}_{measured} \times \hat{\mathbf{b}}^b
$$

where $\hat{\mathbf{b}}^b$ is the estimated magnetic field direction in the body frame.

**Total error:**

$$
\mathbf{e} = \mathbf{e}_a + \mathbf{e}_m
$$

> **Sign check.** The source wrote $\hat{\mathbf{g}}\times\mathbf{a}$ (estimated × measured). With the correction applied as $\boldsymbol{\omega}_{gyro} + K_p\mathbf{e}$, that order pushes the estimate *away* from the measurement. A vector fixed in the world, seen from a body rotating at $\boldsymbol{\omega}$, moves as $\dot{\hat{\mathbf{v}}} = \hat{\mathbf{v}} \times \boldsymbol{\omega}$. Choosing $\boldsymbol{\omega} = \mathbf{a}\times\hat{\mathbf{v}}$ gives
> $$\dot{\hat{\mathbf{v}}} = \hat{\mathbf{v}}\times(\mathbf{a}\times\hat{\mathbf{v}}) = \mathbf{a} - \hat{\mathbf{v}}(\hat{\mathbf{v}}\cdot\mathbf{a}),$$
> the part of $\mathbf{a}$ perpendicular to $\hat{\mathbf{v}}$, so $\hat{\mathbf{v}}$ rotates **toward** $\mathbf{a}$. This matches Mahony et al. (2008) and Madgwick's `MahonyAHRS.c` (`halfex = ay*halfvz - az*halfvy`).

### Step 2: PI Correction

**Proportional:**

$$
\boldsymbol{\omega}_p = K_p\,\mathbf{e}
$$

**Integral** (bias compensation):

$$
\mathbf{b}_{k+1} = \mathbf{b}_k + K_i\,\mathbf{e}\,\Delta t,
\qquad \boldsymbol{\omega}_i = \mathbf{b}
$$

$\mathbf{b}$ converges to **minus** the gyro bias, so that adding it cancels the bias.

**Corrected angular velocity:**

$$
\boldsymbol{\omega}_{corrected} = \boldsymbol{\omega}_{gyro} + \boldsymbol{\omega}_p + \boldsymbol{\omega}_i
$$

### Step 3: Quaternion Integration

$$
\dot{\mathbf{q}} = \frac{1}{2}\,\mathbf{q} \otimes \begin{bmatrix} 0 \\ \boldsymbol{\omega}_{corrected} \end{bmatrix},
\qquad
\mathbf{q}_{k+1} = \mathbf{q}_k + \dot{\mathbf{q}}\,\Delta t
$$

**Normalize:**

$$
\mathbf{q}_{k+1} \leftarrow \frac{\mathbf{q}_{k+1}}{\lVert \mathbf{q}_{k+1} \rVert}
$$

This form assumes $\boldsymbol{\omega}$ is expressed in the **body** frame and $\mathbf{q}$ rotates body-frame vectors into the world frame.

---

## Mathematical Details

### Error Calculation

The cross product between measured and estimated directions gives the rotation axis:

$$
\mathbf{e} = \mathbf{v}_{measured} \times \hat{\mathbf{v}}
$$

- **Magnitude**: proportional to the sine of the angle between the (unit) vectors, $\lVert\mathbf{e}\rVert = \sin\theta$
- **Direction**: axis of the rotation needed to align them

### PI Controller

**Proportional term** ($K_p$):

$$
\boldsymbol{\omega}_p = K_p\,\mathbf{e}
$$

- Immediate correction
- Higher $K_p$ = faster response, more noise

**Integral term** ($K_i$):

$$
\mathbf{b}_{k+1} = \mathbf{b}_k + K_i\,\mathbf{e}\,\Delta t
$$

- Accumulates error over time
- Compensates for gyroscope bias
- Higher $K_i$ = faster bias learning, but more overshoot; lower $K_i$ = slower but more stable

### Corrected Angular Velocity

$$
\boldsymbol{\omega}_{corrected} = \boldsymbol{\omega}_{gyro} + K_p\,\mathbf{e} + \mathbf{b}
$$

---

## Parameter Tuning

### $K_p$ (Proportional Gain)

**Effect**: controls response speed to orientation errors.

- **High $K_p$ (5–10)**: fast correction, sensitive to noise
- **Medium $K_p$ (1–3)**: balanced (recommended)
- **Low $K_p$ (0.1–0.5)**: slow correction, smooth output

**Reference defaults** (Madgwick's `MahonyAHRS.c`): $K_p = 0.5$, $K_i = 0$; tune from there.

### $K_i$ (Integral Gain)

**Effect**: controls gyroscope bias compensation.

- **High $K_i$ (0.1–1)**: fast bias compensation, can overshoot
- **Medium $K_i$ (0.01–0.05)**: balanced (recommended)
- **Low $K_i$ (0.001–0.01)**: slow bias compensation, stable
- **Zero $K_i$**: no bias compensation (pure complementary filter)

**Rule of thumb (from the source)**: $K_i \approx 0.1 \times K_p$.

**Check from a small-angle analysis (added).** For small tilt error the loop behaves like a second-order system:

$$
s^2 + K_p\,s + K_i = 0
$$

- Real, non-oscillating poles need $K_i \le K_p^2/4$.
- The slow pole sits near $-K_i/K_p$, so the bias is learned over a time constant of about $K_p/K_i$. For $K_p = 1$, $K_i = 0.02$ that is about 50 s.
- Until the bias is learned, the steady-state tilt error is about $b/K_p$ rad. A 0.03 rad/s bias gives about 1.7°. Calibrate the gyro at boot instead of relying on $K_i$.

---

## Mahony vs Madgwick

| Feature                | Mahony                    | Madgwick                     |
| ---------------------- | ------------------------- | ---------------------------- |
| **Approach**           | Complementary filter + PI | Gradient descent             |
| **Parameters**         | $K_p$, $K_i$ (2 params)   | $\beta$ (1 param)            |
| **Bias Compensation**  | Yes (integral term)       | Yes (gyro bias drift term $\zeta$) |
| **Computational Cost** | Lower                     | Slightly higher              |
| **Tuning Complexity**  | Moderate                  | Easier                       |
| **Convergence**        | Faster with good $K_i$    | Depends on $\beta$           |
| **Stability**          | Very stable               | Stable                       |

## Advantages

1. **Gyroscope bias compensation**: the integral term removes drift
2. **Computationally efficient**: no matrix operations
3. **Stable**: the PI controller is well understood
4. **Flexible**: set $K_i = 0$ for a pure complementary filter
5. **Real-time capable**: fast enough for embedded systems

---

## Limitations

1. **Two parameters**: $K_p$ and $K_i$ need tuning (vs Madgwick's single $\beta$)
2. **Assumes the accelerometer measures only gravity**: wrong during linear acceleration (mitigated by gating, see below)
3. **Integral windup**: $K_i$ can accumulate error during prolonged disturbances
4. **Yaw drift**: without a magnetometer, yaw is unobservable and drifts over time. Gravity only constrains roll and pitch.

---

## Applications

- **Drones/Quadcopters**: attitude control with bias compensation
- **Robotics**: IMU-based navigation
- **VR/AR**: head tracking with drift correction
- **Wearables**: activity and motion tracking
- **Stabilization**: camera gimbals, platforms

---

## Best Practices

1. **Start with $K_p$ only**: set $K_i = 0$, tune $K_p$ for the desired response
2. **Add $K_i$ gradually**: increase $K_i$ until the bias is compensated
3. **Monitor the integral term**: reset it if it grows too large
4. **Calibrate sensors**: remove static bias before filtering
5. **Handle saturation**: limit integral accumulation
6. **Test with motion**: verify performance during actual use

---

## Conventions used in this project (`gimbal_fw`)

These pin down the generic equations above to the code in `mahony_ref.py` and `mahony.c`.

- $\mathbf{q}$ is $\mathbf{q}_{wc}$: it rotates **camera-frame** vectors into the **world** frame, so $R_{wc} = R(\mathbf{q})$.
- The world frame is z-down (FRD-like). The accelerometer measures *specific force*, which points **up** when at rest, i.e. $(0,0,-1)\,\text{g}$ in the world frame. The estimate must use the same quantity, so $\hat{\mathbf{v}}$ is the predicted **specific force** direction in the camera frame, $\hat{\mathbf{v}} = -R_{wc}^{T}\,[0,0,1]^T$. As a quaternion expression (the negated third row of $R_{wc}$):

$$
\hat{\mathbf{v}} = -\begin{bmatrix}
2(q_x q_z - q_w q_y) \\
2(q_y q_z + q_w q_x) \\
q_w^2 - q_x^2 - q_y^2 + q_z^2
\end{bmatrix}
$$

- Error: $\mathbf{e} = \dfrac{\mathbf{a}}{\lVert\mathbf{a}\rVert} \times \hat{\mathbf{v}}$. The accelerometer is normalised first, because only its direction carries attitude.
- **Gating**: skip the accelerometer correction ($\mathbf{e} = \mathbf{0}$) when $\left|\,\lVert\mathbf{a}\rVert - 1\,\text{g}\,\right| > 0.15\,\text{g}$. The bias term $\mathbf{b}$ keeps being applied while gated.
- Gyro and accelerometer vectors are first rotated into the camera frame with $R_{imu\_cam}$ (must satisfy $\det = +1$) before entering the filter.
- Start values: $K_p \approx 1.0$, $K_i \approx 0.02$.
