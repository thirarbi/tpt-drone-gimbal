# Bab 4.2 -- Hasil Eksperimen

| # | Risk question | Metric / criteria | Result | Source | Pass |
|---|---------------|-------------------|--------|--------|------|
| 1 | Can the perception+control loop close within the photon-to-setpoint budget? | projected full p95 vs 66 ms ceiling | projected p95 17.85 ms (ceiling 66.0 ms; compute 1.85 ms) | SYNTHETIC_MODEL | PASS |
| 2 | Is the nested pad detectable across the whole AGL band (incl. handoff)? | min detection rate vs 95% threshold | min 100% (thr 95%); failing [] m; src crossover ~4.75 m | SYNTHETIC_MODEL | PASS |
| 3 | Would rolling shutter corrupt pose during gimbal slew (vs chosen global)? | rolling-shutter corner error at SEARCH slew | global 0 px (chosen HW); rolling 1.717 px @ 60.0 deg/s vs 1.0 px budget (would exceed) | SYNTHETIC_MODEL | PASS |
| 4 | Does the PnP pose-flip ambiguity corrupt the landing position near nadir? | worst-case position impact vs 5 cm (+ reproj/chi2 discrimination) | position impact 0.113 cm (< 5.0 cm); flip is rotational, chi2-reject 0%; orient-ambiguous [] deg | SYNTHETIC_MODEL | PASS |
| 5 | Does the BEC have headroom for the Pi 5 + payload worst case? | peak current vs rated current | peak 3.63 A / rated 5.0 A; margin 1.37 A | SYNTHETIC_MODEL | PASS |
| 6 | Does velocity feedforward keep the vehicle over a moving pad? | steady tracking error with FF on | 4.95 cm FF-on vs 111.79 cm FF-off | SYNTHETIC_MODEL | PASS |

**Overall: PASS** (6/6 criteria met)

---

## Minimum criteria to succeed + safety margins

These are the binding conditions for each risk question and the headroom the
current design holds over them. If a condition is violated, the corresponding row
flips to FAIL.

| # | Minimum required to pass | Current design | Safety margin |
|---|--------------------------|----------------|---------------|
| 1 | photon-to-setpoint p95 <= 66 ms ceiling | projected p95 ~17.8 ms | ~48 ms (~73% of budget unused) |
| 2 | every AGL band detects >= 95%; needs outer marker big enough to clear the ~20 px detector floor at 30 m **and** inner marker + quiet ring inside the FOV at 0.30 m, **and** the approach holds the pad within <= 10% of AGL off-nadir | min band 100%; outer 1.25 m -> ~23 px @ 30 m; inner 0.10 m -> ~1.2 cm (~22 px) FOV clearance @ 0.30 m; handoff/crossover ~4.75 m sits mid-band | +5 pp over the 95% floor; ~17% over the pixel floor at 30 m; ~11% FOV clearance at 0.30 m |
| 3 | pose-corner error <= 1.0 px during the 60 deg/s SEARCH slew | global shutter -> 0 px (rolling would be ~1.7 px) | full 1.0 px budget unused; choosing global shutter is the load-bearing requirement |
| 4 | pose-flip position impact <= 5.0 cm, gated by reproj (<=3 px) + chi2 | worst-case ~0.11 cm (the flip is rotational, not translational) | ~44x (4.89 cm) |
| 5 | BEC headroom >= 1.0 A over the Pi 5 + payload peak | peak 3.63 A on a 5.0 A rail | 1.37 A (27% of rated) |
| 6 | steady moving-target error <= 10 cm with feedforward on | 4.95 cm (off: 112 cm) | ~2x (5.05 cm); feedforward must be enabled |

**Load-bearing assumptions for row 2 (detection):** lo-res 640x400 detect stream
(f = 560 px), nested pad of a 1.25 m outer + 0.10 m inner marker, and a precision
approach that keeps the pad within ~0.1 x AGL of nadir. At the original 18%-of-AGL
worst-case offset, a two-marker pad cannot span 0.3-30 m on the lo-res stream (the
outer overflows the frame before the inner is large enough at the handoff); the
fix was to size both markers to the band ends and bound the offset envelope, not
to add hardware.
