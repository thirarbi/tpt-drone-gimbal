

### The 7 Design Principles

1. Latency is the benchmark. The system budgets photon-to-setpoint latency (target 33 ms, ceiling 66 ms), not just camera FPS. It's set on `config.LATENCY_ALLOC_MS`, `latency_bench.py`. 
2. **Velocity feedforward**. `v_cmd = Kp.error G_ff.v_target_absolute`. Written in `landing.py`, `config.FEEDFORWARD_GAIN`.

### Reading `config.py`
There are no bare numeric literals anywhere else in the codebase, except here. Everything you tune is in this file, tagged with provenance:
- `# MEASURE`: Must be confirmed against real hardware / calibration (e.g. the camera matrix `CAM_K`,`EKF_MEAS_POS_STD`)
- `# TYPICAL`: Datasheet/handbook stand-in until measured.
- `# DESIGN`: Chosen design value.
