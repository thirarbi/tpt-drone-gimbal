# What `gen_golden*` is, and why the plan needs it

## The problem it solves

The plan has the same maths in two languages:

| Math | Python (reference, on the Pi/laptop) | C (runs on the ESP32) |
|---|---|---|
| Kinematics (FK/IK) | `landing_sw/gimbal_kinematics.py` | `gimbal_math/kinematics.c` |
| Attitude filter (Mahony) | prototype in T5 | `gimbal_math/mahony.c` |

Python is where you can derive, plot and sanity-check. C is where the maths actually runs on the gimbal. If the two disagree, you get silent bugs, such as a swapped sign or a wrong rotation order. The plan says these are the things to always test first (§0): "silent or expensive when wrong", and lab time counts as expensive. §7 lists the risk as "C and Python maths drift apart".

## What "golden" means

A **golden vector** is a known-good input → output pair, produced by a trusted reference. Here, Python is the reference.

`gen_golden.py` is a script that runs your Python implementation on many inputs and writes the results to a CSV. For example, `golden_kinematics.csv` holds rows of `(q, R)`: joint angles and the rotation matrix Python computed from them (T3a).

The C host test (`host_test/test_kinematics.c`, T3b) then does the following:

1. Read each row of the CSV.
2. Feed `q` into the C `gimbal_fk()`.
3. Compare the result to Python's `R`, with a tolerance of 1e-5 on the matrix entries.
4. Do the same for IK, comparing with `wrap_pi(a−b)` so ±π doesn't cause false failures.

## Why a CSV and not just "run both"

- **Decoupling.** The C test only needs `gcc` and a file. It doesn't need Python or the ESP32.
- **Frozen reference.** The CSV is committed, so a later change to the Python can't silently change what the C is tested against.
- **Edge cases are deliberate.** The generator can include angles at ±π, at the joint limits, and near the roll singularity, as §T3a suggests. Random inputs alone wouldn't reliably cover them.

## The new `gen_golden_mahony.py`

This is the same idea applied to T5. Per the plan: "host test: C on the same log == Python output (1e-4)". The files in your git status fit that:

- `mahony_ref.py` is the Python reference filter.
- `gen_golden_mahony.py` feeds it a sequence of gyro/accel samples and records the quaternion or rotation after each step.
- A future C test replays the same samples through `mahony.c` and compares.

The filter is *stateful* (it integrates over time), so a mismatch compounds. A tiny difference in step 1 can become a visible drift by step 10 000. That's why the golden file should be a sequence, not isolated points.

## How to think about it

When you port code between languages, don't trust yourself to eyeball the equivalence. Make the old implementation produce data, and make the new one reproduce it. If a test fails, the reference is right *by definition*, so the bug is in the port. That's also why §0 says to verify numerically "never by reading".

If you want, I can walk through `mahony_ref.py` and `gen_golden_mahony.py` with you and explain what each section of the generator is covering. I'd leave the writing to you, per the plan.