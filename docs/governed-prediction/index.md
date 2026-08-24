---
layout: default
title: Governed Prediction
---

# Governed plant correlation and laptime prediction

The production workflow preserves the distinction between reproducing one log
and predicting a vehicle intervention:

1. `lts.app.preprocess_plant_data` extracts one complete run.
2. The dataset catalog assigns complete runs to calibration,
   baseline-validation, or transport-validation roles.
3. `lts.app.calibrate_plant` uses cataloged calibration runs and
   `lts.calibration.StagedCalibrator` to identify only manifest parameters
   marked `global_calibrated`, one physically motivated stage at a time.
4. `lts.app.run_plant_validation` predicts a complete run from a single initial
   state and reports channel residuals and systematic trends.
5. `lts.app.predict_design_change` applies a provenance-backed change without
   refitting and runs the same hierarchical optimizer policy for baseline and
   variant.

## Quick Predict

```matlab
addpath('src')
result = lts.app.predict_design_change( ...
    'Change', 'config/design_changes/example_ballast_removal.json', ...
    'Track', '2026enduro', ...
    'AllowProvisional', true)
```

`AllowProvisional=true` explicitly opts into an uncertified engineering study;
it does not promote the calibration. A close replay fit is not by itself
evidence that a modified vehicle is predicted correctly: production studies
accept only a governed parameter manifest and a non-legacy calibration
artifact, and a changed vehicle remains `uncertified` until a known real-car
A/B intervention passes the transport-validation gate. The supplied R25
artifact is intentionally `provisional`.

## Parameter roles

- `design`: permitted vehicle interventions; prohibited from calibration.
- `fixed_measured`: direct evidence; prohibited from calibration.
- `global_calibrated`: invariant physical unknowns that may be identified.
- `run_nuisance`: explicit test-day or scenario conditions.
- `legacy_effective`: historical error-absorbing fits, prohibited from
  production studies.

The R25 manifest and provisional artifact live under `config/governance`.
Design changes are structured JSON documents. Mass operations require mass and
installed position so CG, weight distribution, and yaw inertia change together.
Aero operations require a complete map rather than an isolated multiplier.

## Certification

`provisional` means baseline evidence only. `baseline-validated` requires
held-out unchanged-car runs. `transport-validated` additionally requires a
known A/B intervention with no variant refit. The real-car gate is:

- absolute lap-time error at most 2%;
- delta direction correct;
- delta error at most `max(20% of measured delta, 0.2 s)`.

Until that gate passes, design results are explicitly marked `uncertified`.
