---
layout: page
title: Correlation Replay
permalink: /correlation/
---

# Correlation replay reference

`lts.app.run_correlation` replays real MoTeC controls through the simulator:
it extracts the measured throttle, brake, steer, and starting speed from a
selected lap (or the whole log when `Lap` is omitted), uses those controls
instead of `lts.driver.DriverModel`, then exports a new simulated CSV/`.ld` for
overlay in MoTeC i2.

```matlab
addpath('src')
lts.app.run_correlation( ...
    'MoTeCFile', 'data/real_run.ld', ...
    'Lap', 4, ...
    'VehicleConfig', @lts.vehicles.R25, ...
    'TuningFile', 'R25_correlation_tuning', ...
    'Track', '2026enduro')
```

This page collects the conventions and overrides for normalized replay. For the
data-flow view (extraction → profile → initializer → simulator), see
[Data Ingestion](../data-ingestion/).

## Replay is free-space input replay

Correlation replay uses the imported driver inputs and initial vehicle state,
then lets the physics model produce the path. There is **no** GPS/course
alignment, track rebasing, path projection, or track-limit stop by default. The
configured track is still loaded for export metadata, but its geometry does not
steer or constrain the car. Replays default to the time domain and stop at the
imported replay duration, not the reference track end; use `ReplayDomain`,
`StopAtReplayEnd`, and `StopAtTrackEnd` to override timing behavior. Console
progress and control-input plots use the replay stream's source time/distance.

Logged yaw rate is used as the initial yaw rate by default when present. Pass
`correlation.initialTransientWindowS` in a tuning overlay, or
`InitialTransientWindowS` to `run_correlation`, to seed yaw rate and lateral
transient channels from a local linear estimate at the replay boundary instead
of a single noisy sample. The fit is capped at 50 ms so later transient
evolution cannot be folded into the initial state. When front and rear
accelerometers are present, their rigid-body relationship reconstructs CG lateral
acceleration and initial yaw acceleration.

## Channel maps

Lap slicing uses 1-based public lap numbers and the matching `.ldx` sidecar when
`Lap` is supplied. Omit `Lap` for logs that already contain one run, such as
autocross.

`run_correlation` defaults to `config/motec/r25_real_channel_map.json`, which
scales and sign-flips the R25 real logger's `Steering.Angle` channel while
leaving simulator-exported `Steer Raw` as direct road-wheel angle. The generic
map remains at `config/motec/default_channel_map.json`; copy either map for a
specific logger or car if the channels use different names, signs, units, or
calibration. The normalized replay CSV and extraction manifest are written
beside the simulated output under `exports/correlation_*`.

### Visualizing a correlation run

The `external/LTSTelemetryVisualizer` submodule turns a correlation run
into a standalone animated 3D replay — simulation and aligned real cars on
the reference track with chase/cockpit/orbit/top cameras, playback and
scrubbing, live pedal/steer telemetry, and click-to-seek strip charts. The
quickest entry point is:

```matlab
addpath('external/LTSTelemetryVisualizer')
result = ltsviz.render3D( ...
    'SimCsv', 'exports/correlation_run.csv', ...
    'RealMoTeCFile', 'data/lap5_raw.ld', ...
    'TrackFile', 'tracks/endurance_track_grid_25ft_from_matlab_smoothed.mat');
web(result.htmlFile)
```

`scripts/visualize_correlation.m` wraps the fuller
`ltsviz.visualizeCorrelation` flow, which additionally writes the Plotly
correlation report, the aligned CSV, and the summary JSON beside the 3D
replay (`<output>_3d.html`). See the submodule README for viewer controls
and deep links (`?t=..&cam=..`).

### Brake modes

If the log has no direct brake pedal channel, the default map derives
`brake_ratio` from `Brake Pressure Front` and `Brake Pressure Rear`: it converts
both pressures to bar, sums the front and rear pressure traces, maps the peak
combined pressure in the imported log/window to `1.0`, and scales the remaining
samples by the same peak.

The extractor carries `brake_pressure_front_bar` and `brake_pressure_rear_bar`
into the replay CSV, and the simulator converts them through the vehicle's
brake-pressure calibration instead of the peak-normalized `brake_ratio` path.
An explicit `'BrakeMode', 'ratio'` or `'BrakeMode', 'pressure'` argument
overrides the tuning default.

## Powertrain modes

Replay defaults to `PowertrainMode = "throttle"`, which drives the simulated
powertrain from logged throttle through the throttle map and EMRAX torque
envelope. Two direct modes bypass that map:

- `'PowertrainMode', 'motor_torque_command'` — treats `motor_torque_command_nm`
  as a signed motor-side request, bypassing the throttle map and torque
  envelope.
- `'PowertrainMode', 'motor_torque_delivered'` — **preferred for R25
  correlation.** Uses the measured signed shaft torque directly and applies only
  `powertrain.deliveredTorqueDrivetrainEfficiency` between motor shaft and
  driven axle, deliberately bypassing request-side pack-power and RPM limits.

Positive commands are drive torque; negative commands are sent through the
driveline as regen/decel torque. Regen uses the inverse drivetrain-loss
direction, so wheel-side braking power is larger in magnitude than the power
recovered at the pack by the configured regen efficiency. Motoring torque uses
`powertrain.efficiency`; negative direct-mode regen uses
`powertrain.regenEfficiency` when present, falling back to
`powertrain.efficiency`.

### Pack-power limiting

When `pack_voltage_v` and `pack_current_a` are present, the simulator caps the
applied motor torque so requested motor mechanical power cannot exceed the
logged DC pack power before reflecting the capped torque through drivetrain
efficiency and final-drive ratio. Pass `'LimitMotorTorqueByPackPower', false` to
disable the cap for diagnosis. Pack current is positive for
discharge/motoring power and negative for charging/regen power.

The cap uses logged `motor_rpm` when available, then logged replay speed, and
only falls back to simulated speed when no measured speed source exists. If
`Calculated Cmd` has already crossed back positive while the pack is still
charging, the decoded `regen_torque_nm` request supplies the negative direct-mode
request and the pack-power cap still limits the applied value. If measured pack
voltage/current lag the inverter command in the raw log, pass
`'PackPowerAdvanceS', seconds` to advance only those pack channels before replay
sampling (a positive value, e.g. `0.06`, pulls pack samples 60 ms earlier; the
default is `0`).

You can also pass `'CorrelationConfig', 'path/to/config.json'`; configs written
by `scripts/extract_correlation_config.py` currently provide `PackPowerAdvanceS`
for replay and `GpsAdvanceS` for GPS-position overlay work.

## R25 inverter and torque channels

For R25 logs, the extractor also carries:

- `regen_torque_nm` from `Throttle Regen Negative Torque Command` /
  `Throttle Regen Negative Torque C`;
- `motor_torque_command_nm` from `BAMOCAR Channels Calculated Cmd`;
- `motor_torque_delivered_nm` from `BAMOCAR Channels Iq`;
- `motor_rpm` from `BAMOCAR Channels RPM`;
- `pack_voltage_v` / `pack_current_a` from the BMS.

These raw torque channels are stored with unsigned 16-bit conventions even
though the physical signals are signed; the channel map wraps `65536` back to
zero, decodes signed storage before scaling, and `regen_torque_nm` is exported
on the negative-torque side for correlation. The always-negative
`regen_torque_nm` capability channel is used only when the signed motor-command
channel is unavailable; it does not by itself indicate that regen is active.

### Decoding `BAMOCAR Channels Iq`

The raw `BAMOCAR Channels Iq` signal is a unitless integer CAN quantity, not an
engineering-Apeak channel. Lap-integrated power conservation identifies its
conversion as `0.5 Arms/count`. For the EMRAX 228 medium-voltage winding,
delivered torque is therefore computed as
`Iq * 0.5 Arms/count * 0.48 Nm/Arms = Iq * 0.24 Nm/count`.

### Regen power-conservation repair

The extractor repairs physically impossible regen samples after aligning the
pack channels 15 ms later than motor torque (this alignment minimizes the
reconstructed braking impulse). While the signed motor command requests regen
and the pack is charging, shaft mechanical input power cannot be lower than
recovered DC power. Iq samples that contradict the charging sign or violate that
bound beyond 500 W are replaced by the minimum shaft torque required at 100%
conversion efficiency. This is a conservative power-conservation bound:
coherent Iq samples remain unchanged, and no fitted regen-efficiency multiplier
is applied. Repair counts and limits are recorded under
`postprocessing.delivered_regen_power_repair` in the extraction manifest.

The supplied `R25_correlation_tuning` applies the same idempotent check after
MATLAB loads a replay profile. Consequently `run_correlation` repairs both newly
extracted MoTeC data and older files passed through `ReplayCsv`; the returned
`outputs.regenRepairSampleCount` and related fields report what was changed.
This runtime check is enabled only for
`PowertrainMode='motor_torque_delivered'`.

## Tuning overlays

For correlation-only setup changes, keep `lts.vehicles.R25` as the base vehicle
and pass a tuning overlay with `TuningFile` or `VehicleTuning`. The supplied
`src/+lts/+vehicles/R25_correlation_tuning.m` overlay applies lap5/raw drivetrain
assumptions without changing the base car definition. Surface friction is fixed
at unity throughout the simulation; the legacy `SurfaceMu` replay option remains
accepted for compatibility but is ignored.

### Steering transfer curve (R25)

The R25 overlay corrects the steering sensor's known center-region
nonlinearity with a quadratic transfer curve. It preserves the measured
road-wheel angle at ±22°, applies a slope and offset at center, fades the offset
to zero at both endpoints, and delays the corrected command to match the
measured whole-lap yaw response. Override independently with:

- `SteeringCenterGain` (slope at center; `1` disables the correction);
- `SteeringCenterOffsetRad` (offset at center; `0` disables);
- `SteeringCalibrationEndAngleRad` (endpoint where the offset fades to zero);
- `SteeringDelayS` (command delay in seconds; `0` disables).

### Wheel speed seeding

Set `correlation.useLoggedWheelSpeeds` in a tuning overlay, or pass
`UseLoggedWheelSpeeds`, to choose between logged corner speeds and a
zero-initial-slip wheel-speed state derived from the local contact-patch
kinematics. Optional `wheel_speed_fl_mps`, `wheel_speed_fr_mps`,
`wheel_speed_rl_mps`, and `wheel_speed_rr_mps` columns seed the initial
per-corner tire angular velocity; missing wheel-speed sensors fall back to the
median of the valid corners at the first sample.

### Tire and wheel assumptions (R25)

The scaled 43075 tire uses an effective longitudinal stiffness scale `LKX`,
calibrated against driven-wheel carrier speed reconstructed from measured motor
RPM, the specified final drive, and the physical rolling radius. Treat the
configured value as an effective scale for the inherited tire coefficients, not
as a measured standalone tire property. The R25 wheel dynamics use a
rotating-corner assembly assumption: treating the measured tire as an annulus
and conservatively placing the remaining assembly mass at the rim radius gives
the configured per-corner inertia.

The measured-shaft-torque replay path does **not** apply an additional fitted
torque-proportional drivetrain loss. Wheel/rotor inertia, rolling resistance,
and aero drag remain active; a shaft-to-axle loss should only be restored when
it is supported by direct measurement. The overlay uses an effective
correlation `CdA` that includes unmodeled rotating/drivetrain losses and does
not modify the physical aero definition in `lts.vehicles.R25`.

## Legacy ML-assisted tuning

`lts.app.tune_correlation` is retained only to reproduce historical diagnostic
work. It requires `LegacyDiagnostic=true`, and its generated overlays cannot be
used by governed design studies. It splits a normalized replay into alternating
anchor blocks and predicts 3, 6, and 12 seconds from every anchor. All horizons
at an anchor stay in the same split, preventing overlap between training and
validation. It warm-starts the existing replay pipeline at every window and
scores the GPS trace, GPS-derived speed and body accelerations, yaw rate, and
valid wheel-speed references. GPS-derived motion carries 75% of the objective;
wheel speeds remain an independent drivetrain diagnostic. An Extra Trees model
proposes new bounded physical configurations while held-out windows remain
unused until the finalist stage.

```matlab
addpath('src')
result = lts.app.tune_correlation( ...
    'LegacyDiagnostic', true, ...
    'MoTeCFile', 'data/lap5_raw.ld', ...
    'HorizonS', [3 6 12], ...
    'MaxHours', 8, ...
    'MaxCandidates', 1200, ...
    'Workers', 8)
```

`MoTeCFile` runs the existing MoTeC extraction pipeline automatically. Use `Lap`
for a lap number or inclusive range when the LD contains multiple laps, and
optionally override `LdxFile`, `ChannelMap`, or `ImportFrequency`. The
normalized CSV, extraction manifest, and a hash-verified source descriptor are
saved in the tuning checkpoint. On resume, changed LD/LDX data or extraction
settings are rejected instead of being mixed with existing scores. `ReplayCsv`
remains available when normalized data already exists; do not pass both input
forms.

When finite latitude and longitude are present, tuning converts them to a
smoothed local east/north trace and derives vehicle speed, longitudinal
acceleration, and lateral acceleration from that trace. This GPS-derived body
motion supersedes wheel-speed-derived vehicle speed and the body accelerometer
channels for scoring; the original axle accelerometers and wheel-speed channels
remain available as lower-weight checks. Adjust `GpsSmoothingS` (default
`0.35`) for the GPS receiver rate, or pass `PreferGpsKinematics=false` to retain
the legacy channel hierarchy. Mixed horizons default to `[3 6 12]`. Changing
horizons or GPS preprocessing requires a new checkpoint directory.

The default search space is `config/correlation/lap5_ml_parameter_space.json`.
Each batch is checkpointed under `exports/legacy/correlation_tuning_lap5_*`;
rerun with the same `CheckpointDirectory` to resume. The result folder contains
candidate and per-window scores, held-out rankings, convergence/importance
plots, a hashed JSON manifest, and an apply-ready `R25_ml_lap5_tuning.m`
overlay. The existing `R25.m` and `R25_correlation_tuning.m` files are never
rewritten.
