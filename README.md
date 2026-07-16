# FSAE Transient Lap Time Simulation

An object-oriented MATLAB lap-time simulation framework for FSAE vehicles. The project composes swappable aero, suspension, powertrain, tire, and track models, then runs a transient simulation loop through `lts.simulation.Simulator`.

## Quick Start

```matlab
addpath('src')
lts.app.run_simulation
```

Each run writes a MotecLogGenerator-compatible CSV to `exports/motec_<track>_<timestamp>.csv`.
It also uses the `external/MotecLogGenerator` submodule to create
`exports/motec_<track>_<timestamp>.ld` for MoTeC i2.
Set `exportMoTeC = false` in `src/+lts/+app/run_simulation.m` to disable this.

The CSV header includes units in MotecLogGenerator's `Channel Name (unit)`
format, for example `Engine RPM (rpm)`. The exporter also creates fake
`GPS Latitude (deg)` and `GPS Longitude (deg)` channels from the simulated
position trace so MoTeC can show a map path. During `.ld` conversion the
submodule writes MoTeC-compatible display-unit bytes and an M1/pro-enabled log
header so i2 recognizes exported channels as real quantities for math channels
rather than unitless display-only data.

If the submodule is missing after cloning, initialize it with:

```bash
git submodule update --init --recursive
```

The `.ld` conversion uses Python and MotecLogGenerator's dependencies:

```bash
python -m pip install cantools numpy
```

## Correlation Replay

`lts.app.run_correlation` replays real MoTeC controls through the simulator. It extracts
the measured throttle, brake, steer, and starting speed from a selected lap, or
from the whole log when `Lap` is omitted, uses those controls instead of
`lts.driver.DriverModel`, then exports a new simulated CSV and `.ld` for overlay in MoTeC
i2.

```matlab
addpath('src')
lts.app.run_correlation( ...
    'MoTeCFile', 'data/real_run.ld', ...
    'Lap', 4, ...
    'VehicleConfig', @lts.vehicles.R25, ...
    'TuningFile', 'R25_correlation_tuning', ...
    'Track', '2026enduro')
```

Lap slicing uses 1-based public lap numbers and the matching `.ldx` sidecar
when `Lap` is supplied. Omit `Lap` for logs that already contain one run, such
as autocross.
`lts.app.run_correlation` defaults to `config/motec/r25_real_channel_map.json`, which
scales and sign-flips the R25 real logger's `Steering.Angle` channel while
leaving simulator-exported `Steer Raw` as direct road-wheel angle. The generic
map remains at `config/motec/default_channel_map.json`; copy either map for a
specific logger or car if the channels use different names, signs, units, or
calibration. The normalized replay CSV and extraction manifest are written
beside the simulated output under `exports/correlation_*`.

For correlation-only setup changes, keep `lts.vehicles.R25` as the base vehicle and
pass a tuning overlay with `TuningFile` or `VehicleTuning`. The supplied
`src/+lts/+vehicles/R25_correlation_tuning.m` overlay currently applies the
lap5/raw drivetrain assumptions without changing the base car definition.
Surface friction is fixed at unity throughout the simulation. The legacy
`SurfaceMu` replay option remains accepted for compatibility but is ignored.

Correlation replay is a free-space replay: it uses the imported driver inputs
and initial vehicle state, then lets the physics model produce the path without
GPS/course alignment, track rebasing, path projection, or track-limit stops.
The configured track is still loaded for export metadata, but its geometry does
not steer or constrain the car. Logged yaw rate is used as the initial yaw rate
by default when present.
Set `correlation.initialTransientWindowS` in a tuning overlay, or pass
`InitialTransientWindowS` to `lts.app.run_correlation`, to seed yaw rate and
lateral transient channels from the median of the first N seconds instead of a
single initial sample.
Correlation defaults to time-domain replay and stops at the imported replay
duration, not the reference track end; use `ReplayDomain`, `StopAtReplayEnd`,
and `StopAtTrackEnd` to override timing behavior. Console progress and
control-input plots use the replay stream's source time/distance.

If the log has no direct brake pedal channel, the default map derives
`brake_ratio` from `Brake Pressure Front` and `Brake Pressure Rear`. It converts
both pressures to bar, sums the front and rear pressure traces, maps the peak
combined pressure in the imported log/window to `1.0`, and scales the remaining
samples by the same peak.
For correlation runs that should use the logged line pressures directly, pass
`'BrakeMode', 'pressure'`. The extractor also carries
`brake_pressure_front_bar` and `brake_pressure_rear_bar` into the replay CSV,
and the simulator converts them through the vehicle's brake-pressure
calibration instead of the peak-normalized `brake_ratio` path.
Optional `wheel_speed_fl_mps`, `wheel_speed_fr_mps`, `wheel_speed_rl_mps`, and
`wheel_speed_rr_mps` columns seed the initial per-corner tire angular velocity;
missing wheel-speed sensors fall back to the median of the valid corners at the
first sample.
For R25 logs, the extractor also carries `regen_torque_nm` from
`Throttle Regen Negative Torque Command` / `Throttle Regen Negative Torque C`
and `motor_torque_command_nm` from `BAMOCAR Channels Calculated Cmd`, plus
`motor_rpm` from `BAMOCAR Channels RPM` and `pack_voltage_v` / `pack_current_a`
from the BMS. These raw torque channels are stored with unsigned 16-bit
conventions even though the physical signals are signed; the channel map wraps
`65536` back to zero, decodes signed storage before scaling, and
`regen_torque_nm` is exported on the negative-torque side for correlation. Pack
current is treated as positive for discharge/motoring power and negative for
charging/regen power.
By default replay still drives the simulated powertrain from logged throttle.
Pass `'PowertrainMode', 'motor_torque_command'` to bypass the throttle map and
EMRAX torque envelope. In that mode, `motor_torque_command_nm` is treated as a
signed motor-side request. By default the simulator caps the applied motor
torque against measured DC pack power when `pack_voltage_v` and
`pack_current_a` are present before reflecting it through final-drive ratio and
drivetrain efficiency. Pass `'LimitMotorTorqueByPackPower', false` to disable
that cap for diagnosis. Motoring torque uses `powertrain.efficiency`;
negative direct-mode regen uses `powertrain.regenEfficiency` when present,
falling back to `powertrain.efficiency`. When enabled, the cap uses logged
`motor_rpm` when available, then logged replay speed, and only falls back to
simulated speed when no measured speed source exists. Positive commands are
drive torque; negative commands are sent through the driveline as regen/decel
torque. Regen uses the inverse drivetrain-loss direction, so wheel-side braking
power is larger in magnitude than the power recovered at the pack by the
configured regen efficiency. If `Calculated Cmd` has already crossed back
positive while the pack is still charging, the decoded `regen_torque_nm`
request is used as the negative torque request instead; pack-power limiting,
when enabled, still limits the applied value. If measured pack voltage/current
lag the inverter command in the raw log, pass `'PackPowerAdvanceS', seconds` to
advance only those pack channels before replay sampling. A positive value, for
example `0.06`, pulls pack-power samples 60 ms earlier. The default is `0`.
You can also pass `'CorrelationConfig', 'path/to/config.json'`; configs written
by `scripts/extract_correlation_config.py` currently provide `PackPowerAdvanceS`
for replay and `GpsAdvanceS` for GPS-position overlay work.

Edit `trackType` in `src/+lts/+app/run_simulation.m` to switch between:

- `straight10`
- `straight`
- `oval`
- `skidpad`
- `autocross`
- `busstop`
- `slalom`
- `90turn`
- `2026enduro` — loads a `.mat` centerline from `tracks/` (see [Track files](#track-files))

`skidpad` simulates one warmup lap before the timed lap; returned plots and
MoTeC exports contain only the second lap.

## Track files

`2026enduro` and other real circuits are loaded from `.mat` files in `tracks/`
via `lts.components.WaypointTrack.loadMat`. These files are produced by the separate
[`fsae track image tool`](https://github.com/jyjh/fsae-track-image-tool), which
traces a track image into `[x, y]` waypoints.

**Travel direction.** The exporter bakes the requested clockwise/anticlockwise
direction into the ordering of `points_m` and also records it in a `direction`
field. `loadMat` honors that order by default, and an explicit override can be
passed to force a direction:

```matlab
track = lts.components.WaypointTrack.loadMat('tracks/<file>.mat', 'Direction', 'anticlockwise');
```

If the override conflicts with the direction stored in the file (for example
because the file is a **stale copy** that was re-exported the other way), the
waypoints are reversed — keeping the start/finish point fixed — and a warning is
emitted. `lts.app.run_simulation` passes `'Direction', 'anticlockwise'` for the
endurance track and prints the resolved direction at startup, so a wrong or
stale track is obvious immediately. A file with no direction field at all also
warns.

**Updating a track.** The exporter writes to its own `examples/` directory; it
does **not** touch this repo's `tracks/`. After re-running the exporter (with a
changed `Direction` or otherwise), copy the new `.mat` into `tracks/` yourself.
Note that `Direction` only reorders points *inside* the file — the filename is
unchanged — so a re-export silently overwrites the previous output.

## Current Model

- Whole-car aero system: `lts.components.Aero.WholeCarAero` uses a single ClA/CdA and center-of-pressure location from `cfg.aero`.
- Transient chassis platform: `lts.components.Chassis.SimpleChassis` tracks heave, pitch, and separate front/rear roll DOFs coupled by chassis torsional rigidity for chassis-driven corner loads. Telemetry includes front/rear roll angles and roll rates.
- Four-corner transient suspension: `lts.components.Suspension.SuspensionManager` manages one `SimpleSuspension` and `SuspensionState` per corner.
- Table-based suspension and steering geometry: `lts.components.Suspension.SuspensionGeometry` provides camber, toe, motion ratio, steering axis caster/trail/scrub radius/kingpin inclination, and Ackermann steering presets. Positive caster tilts the axis rearward, positive trail places the contact patch behind the kingpin ground point, and positive scrub radius places it outboard.
- EMRAX 228 powertrain: `lts.components.Powertrain.EMRAX228Powertrain` loads `EMRAX228CC Single_4.5.mat`, tracks motor RPM with `PowertrainState`, applies torque falloff above the data endpoint, and enforces a hard RPM cap.
- Supported Pacejka tire model: `lts.components.Tire.PacejkaTire` loads the provided `.tir` file and tracks per-corner tire state, including suspension-derived camber and per-corner slip angles.
- Test tracks: `lts.components.TestTrack` provides straight, oval, skidpad, autocross, busstop, slalom, and 90-turn layouts.
- MoTeC telemetry export: `lts.telemetry.TelemetryExporter.exportToMoTeCLog` writes simulation logs as MotecLogGenerator-compatible CSVs and converts them to MoTeC `.ld` files through the MotecLogGenerator submodule.

## Documentation

Full documentation is available at [jyjh.github.io/lts](https://jyjh.github.io/lts).

- [Architecture & Usage](https://jyjh.github.io/lts/)
- [UML Class Diagram](https://jyjh.github.io/lts/class-diagram/)
- [Simulation Loop](https://jyjh.github.io/lts/simulation-loop/)
- [Physics Flow](https://jyjh.github.io/lts/physics-flow/)
- [Department Workflow](https://jyjh.github.io/lts/workflow/)
- [Data Ingestion](https://jyjh.github.io/lts/data-ingestion/)

## Requirements

- MATLAB R2019b or later
- [MFeval](https://www.mathworks.com/matlabcentral/fileexchange/63618-mfeval) for Pacejka Magic Formula tire evaluation
- The provided EMRAX and tire data files in `src/+lts/+components/+Powertrain` and `src/+lts/+components/+Tire`
- Python 3 with `cantools` and `numpy` for MoTeC `.ld` export through the [MotecLogGenerator](https://github.com/stevendaniluk/MotecLogGenerator) submodule

## License

See [LICENSE](LICENSE) for details.
