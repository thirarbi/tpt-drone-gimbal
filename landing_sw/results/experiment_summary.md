# Bab 4.2 -- Hasil Eksperimen

| # | Risk question | Metric / criteria | Result | Source | Pass |
|---|---------------|-------------------|--------|--------|------|
| 1 | Can the perception+control loop close within the photon-to-setpoint budget? | projected full p95 vs 66 ms ceiling | projected p95 17.58 ms (ceiling 66.0 ms; compute 1.58 ms) | SYNTHETIC_MODEL | PASS |
| 2 | Is the nested pad detectable across the whole AGL band (incl. handoff)? | min detection rate vs 95% threshold | min 57% (thr 95%); failing [0.3, 30.0] m; src crossover ~3.5 m | SYNTHETIC_MODEL | FAIL |
| 3 | Would rolling shutter corrupt pose during gimbal slew (vs chosen global)? | rolling-shutter corner error at SEARCH slew | global 0 px (chosen HW); rolling 1.069 px @ 60.0 deg/s vs 1.0 px budget (would exceed) | SYNTHETIC_MODEL | PASS |
| 4 | Does the PnP pose-flip ambiguity corrupt the landing position near nadir? | worst-case position impact vs 5 cm (+ reproj/chi2 discrimination) | position impact 0.098 cm (< 5.0 cm); flip is rotational, chi2-reject 0%; orient-ambiguous [] deg | SYNTHETIC_MODEL | PASS |
| 5 | Does the BEC have headroom for the Pi 5 + payload worst case? | peak current vs rated current | peak 3.63 A / rated 5.0 A; margin 1.37 A | SYNTHETIC_MODEL | PASS |
| 6 | Does velocity feedforward keep the vehicle over a moving pad? | steady tracking error with FF on | 4.95 cm FF-on vs 111.79 cm FF-off | SYNTHETIC_MODEL | PASS |

**Overall: FAIL** (5/6 criteria met)
