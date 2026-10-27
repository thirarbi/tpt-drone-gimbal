

### The 7 Design Principles

1. Latency is the benchmark. The system budgets photon-to-setpoint latency (target 33 ms, ceiling 66 ms), not just camera FPS. It's set on `config.LATENCY_ALLOC_MS`, `latency_bench.py`. 
2. **Velocity feedforward**. `v_cmd = Kp.error G_ff.v_target_absolute`. Written in `landing.py`, `config.FEEDFORWARD_GAIN`.

### Reading `config.py`
There are no bare numeric literals anywhere else in the codebase, except here. Everything you tune is in this file, tagged with provenance:
- `# MEASURE`: Must be confirmed against real hardware / calibration (e.g. the camera matrix `CAM_K`,`EKF_MEAS_POS_STD`)
- `# TYPICAL`: Datasheet/handbook stand-in until measured.
- `# DESIGN`: Chosen design value.

---
At first I followed along Claude's recommended workflow and mavlink's workflow of building the systems bit by bit and then creating tests on it. However, I've realized this would take too much time, especially on the encroaching deadline and my approach of studying deep on the system. Tests doesn't help this verymuch. So then I changed it to a more "Regression testing" approach of trial and error. If there's a silent or unknown error, I'd try to reproduce and identify it using the tests.

There's a risk to this however, so I will mark the "critical codes", the ones that'd be costly to reimplement / if anything goes wrong to always be tested first. The rest are just marked as junctions where "a test was supposed to be here, but now skipped".