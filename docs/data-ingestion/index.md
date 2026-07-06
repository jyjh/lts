---
layout: page
title: Data Ingestion
permalink: /data-ingestion/
---

## Data Ingestion And Export Architecture

The current project consumes external data files directly through component constructors and exports simulator telemetry for external review. Additional generic loader classes remain planned.

> Maintainer note: The diagram below is generated from [`data_ingestion.mmd`](data_ingestion.mmd). Edit that file, then run `node docs/sync_diagram.js` to regenerate the SVG.

![Data Ingestion Class Diagram](data_ingestion.svg)

---

### Implemented Inputs

| Data | File | Consumed By | Purpose |
|------|------|-------------|---------|
| EMRAX 228 map | `src/+lts/+components/+Powertrain/EMRAX228CC Single_4.5.mat` | `lts.components.Powertrain.EMRAX228Powertrain` | Final-drive ratio, motor RPM curve, motor torque, and tractive force |
| Hoosier tire file | `src/+lts/+components/+Tire/43105_18x7.5_10_R25B_7.tir` | `lts.components.Tire.TireConstants` / `PacejkaTire` | Pacejka Magic Formula coefficients and nominal tire properties |
| Test track layouts | `lts.components.TestTrack` methods | `lts.simulation.Simulator` / `lts.driver.DriverModel` | Track points, curvature, heading, and surface friction |
| Real MoTeC lap | `.ld` plus optional `.ldx` | `scripts/extract_motec_lap.py` / `lts.app.run_correlation` | Driver input replay and starting-state extraction for correlation |

### Powertrain Data Behavior

`EMRAX228Powertrain` loads the MAT file and uses:

- `FDR` for total gear ratio.
- `Gearing_Map.RPM` and `Gearing_Map.Traction` for full-throttle drive-force lookup by motor RPM.
- `Speed`, `Torque`, and `Tractive_force` for compatibility torque lookup and wheel-radius inference.
- `rpmFalloffStartRPM` from the last RPM in the map.
- `rpmFalloffFactor` to shape torque falloff between the map endpoint and `rpmLimitRPM`.

### Telemetry

The current `stateLog` includes:

- Vehicle channels: time, distance, speed, acceleration, curvature, heading.
- Driver inputs: throttle, brake.
- Aero channels: downforce, drag, front/rear aero loads.
- Suspension channels: corner normal loads, damper position, damper velocity.
- Tire channels: slip ratio, wheel angular velocity, tire longitudinal/lateral force.
- Powertrain channels: drive force, motor RPM, motor torque, wheel torque, driven-wheel RPM, RPM limiter state.

`lts.telemetry.TelemetryExporter.writeToMoTeCFormat(stateLog, filepath)` writes this data to a CSV that follows the [`MotecLogGenerator`](https://github.com/stevendaniluk/MotecLogGenerator) CSV input requirements: the first column is time, remaining rows contain numeric samples, and channel headers include units using `Channel Name (unit)` when a unit is known. For example, motor RPM is written as `Engine RPM (rpm)`. Channels without a known unit are exported without a suffix and remain unitless. The exporter also adds convenience channels such as acceleration in g, fake GPS latitude/longitude in degrees, steering/camber/toe in degrees, damper positions in millimeters, slip ratios in percent, and wheel speeds in rpm.

`lts.telemetry.TelemetryExporter.exportToMoTeCLog(stateLog, filepath)` writes the CSV and then invokes `external/MotecLogGenerator/motec_log_generator.py` to create a `.ld` file. The submodule writes MoTeC-compatible 8-byte display-unit fields and an M1/pro-enabled log header so i2 can use exported channels in math expressions with the correct quantities. `src/+lts/+app/run_simulation.m` enables this by default and writes both `exports/motec_<track>_<timestamp>.csv` and `exports/motec_<track>_<timestamp>.ld`.

Manual conversion is available from MATLAB through `lts.telemetry.TelemetryExporter`:

```matlab
addpath('src')
lts.telemetry.TelemetryExporter.convertCsvToMoTeCLog( ...
    'exports/motec_autocross_20260616_153000.csv', 'Frequency', 1000)
```

The submodule and Python dependencies are required for `.ld` conversion:

```bash
git submodule update --init --recursive
python -m pip install cantools numpy
```

### Correlation Replay

`lts.app.run_correlation` adds the inverse path for correlation work:

1. `scripts/extract_motec_lap.py` reads a real `.ld` file through
   `external/MotecLogGenerator/ldparser`.
2. If a `Lap` is supplied, `ldparser` uses BCN markers from the matching `.ldx`
   sidecar to slice the requested 1-based public lap. If `Lap` is omitted, the
   full log is imported, which is useful for autocross and other one-run logs.
3. `config/motec/default_channel_map.json` maps channel names and units to
   the normalized replay contract:
   `time_s`, `distance_m`, `throttle_ratio`, `brake_ratio`,
   `brake_pressure_front_bar`, `brake_pressure_rear_bar`, `steer_rad`, and
   `speed_mps`, with optional yaw, GPS, course, acceleration, and per-corner
   wheel-speed channels.
   `lts.app.run_correlation` defaults to `config/motec/r25_real_channel_map.json`,
   which applies the R25 real logger steering ratio/sign convention while
   preserving direct simulator-exported steering channels.
   If no direct brake pedal channel exists, `brake_ratio` is derived from
   `Brake Pressure Front` and `Brake Pressure Rear`. The default derivation
   converts both channels to bar, sums front plus rear pressure, maps the peak
   combined pressure in the imported log/window to `1.0`, and scales every other
   sample by that same peak.
   Passing `'BrakeMode', 'pressure'` to `lts.app.run_correlation` uses the front and rear
   pressure columns directly with the vehicle brake-pressure calibration instead
   of the peak-normalized `brake_ratio`.
   For R25 inverter channels, `regen_torque_nm` is extracted from
   `Throttle Regen Negative Torque Command` / `Throttle Regen Negative Torque C`
   and `motor_torque_command_nm` from `BAMOCAR Channels Calculated Cmd`. The
   R25 map also carries `motor_rpm` from `BAMOCAR Channels RPM` and
   `pack_voltage_v` / `pack_current_a` from the BMS so direct-command replay can
   be checked against measured DC pack power. The raw torque values use unsigned
   16-bit storage for signed torque concepts; the extractor wraps `65536` back
   to zero, decodes values above 32767 by
   subtracting 65536 before scaling, and exports the throttle-regen channel as
   negative Nm because it represents the regen-side torque candidate/command.
   Pack current is treated as positive for discharge/motoring power and negative
   for charging/regen power.
4. `lts.correlation.CorrelationReplayProfile` validates the normalized CSV and synthesizes
   distance from speed when the log has no lap-distance channel. Initial-state
   import seeds per-corner tire angular velocity from logged wheel speeds when
   available; missing corners fall back to the median of the valid logged wheel
   speeds at the first sample.
5. `lts.correlation.CorrelationTrackAlignment` estimates the start station from GPS true course
   plus yaw-rate/lateral-G curvature shape, rebases closed tracks so the matched
   station is simulation `s = 0`, and strict preflight rejects large heading
   mismatches before the simulator runs.
6. `lts.correlation.TelemetryReplayDriver` feeds those measured controls into `lts.simulation.Simulator.step`
   by distance or by time.
   Replays default to `PowertrainMode = "throttle"`, which uses logged throttle
   through the simulated powertrain model. For inverter-command correlation,
   pass `'PowertrainMode', 'motor_torque_command'`; this requires
   `motor_torque_command_nm`, bypasses the throttle map and motor torque
   envelope, and treats the signed channel as a motor-side request. When
   `pack_voltage_v` and `pack_current_a` are present, the simulator caps the
   applied motor torque so requested motor mechanical power cannot exceed the
   logged DC pack power before reflecting the capped torque through drivetrain
   efficiency and final-drive ratio before the differential split. Motoring
   torque uses `powertrain.efficiency`; negative direct-mode regen uses
   `powertrain.regenEfficiency` when present, falling back to
   `powertrain.efficiency`. The cap uses logged `motor_rpm` when available,
   then logged replay speed, and only falls back to simulated speed when no
   measured speed source exists. Negative regen torque uses the inverse
   drivetrain-loss direction, so wheel-side braking power is larger in magnitude
   than recovered pack power by the configured regen efficiency. If
   `Calculated Cmd` has already crossed back positive while measured pack power
   is still charging, the decoded `regen_torque_nm` request supplies the
   negative direct-mode request and the pack-power cap still limits the applied
   value.
7. `lts.telemetry.TelemetryExporter.exportToMoTeCLog` writes the simulated replay as a new
   `.csv` and `.ld` for direct comparison in MoTeC i2.

Correlation-specific vehicle assumptions can be layered on top of the base car
with `TuningFile` or `VehicleTuning`. This keeps `lts.vehicles.R25` as the source
vehicle and moves lap-specific drivetrain investigation into an overlay such as
`src/+lts/+vehicles/R25_correlation_tuning.m`.

Example:

```matlab
addpath('src')
lts.app.run_correlation( ...
    'MoTeCFile', 'data/real_run.ld', ...
    'Lap', 4, ...
    'VehicleConfig', @lts.vehicles.R25, ...
    'TuningFile', 'R25_correlation_tuning', ...
    'Track', '2026enduro')
```

Use `StartStation`, `AlignmentDistanceM`, `AlignmentStepM`, and
`StrictPreflight` to tune or override automatic alignment. Logged yaw rate is
imported for alignment and diagnostics but is not used as initial yaw rate by
default; pass `UseLoggedYawRate`, `true` only after its sign convention is
verified. Correlation replay logs off-track status and track-limit margin but
continues by default; pass `StopOnOffTrack`, `true` to stop like a normal
lap-time run. It also defaults to time-domain input replay and stops at the
imported replay duration rather than the reference track end; use
`ReplayDomain`, `StopAtReplayEnd`, and `StopAtTrackEnd` to override that.
Replay progress and control-input plots use the input stream's source
time/distance, not the simulated vehicle's projected reference-track station.

### Interfaces

| Class | Purpose | Status |
|-------|---------|--------|
| `TrackDataLoader` | Load GPS/cone CSV data, smooth curvature, generate racing line | Planned |
| `lts.telemetry.TelemetryExporter` | Export `stateLog` to MotecLogGenerator CSV and convert to MoTeC `.ld` | Implemented |
| `lts.correlation.CorrelationReplayProfile` | Normalized real-lap replay profile with time/distance sampling | Implemented |
| `lts.correlation.CorrelationTrackAlignment` | Start-station estimation and closed-track rebasing for real-lap replay | Implemented |
| `lts.correlation.TelemetryReplayDriver` | Driver adapter that supplies measured inputs to `lts.simulation.Simulator` | Implemented |
| `lts.correlation.CorrelationAppSupport` | App-level loading, tuning, extraction command, and correlation preflight helpers | Implemented |
| Generic aero map loader | Populate aero lookup tables from CFD data | Planned |
