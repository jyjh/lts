# FSAE Transient Lap Time Simulation

An object-oriented MATLAB lap-time simulation framework for FSAE vehicles. The project composes swappable aero, suspension, powertrain, tire, and track models, then runs a transient simulation loop through `lts.simulation.Simulator`.

## Quick Start

```matlab
addpath('src')
lts.app.run_simulation
```

Each run writes a MotecLogGenerator-compatible CSV to `exports/motec_<track>_<config.name>_<timestamp>.csv`
and, through the `external/MotecLogGenerator` submodule, a `.ld` for MoTeC i2.
Set `exportMoTeC = false` in `src/+lts/+app/run_simulation.m` to disable `.ld` conversion.

The CSV header uses MotecLogGenerator's `Channel Name (unit)` format (e.g.
`Engine RPM (rpm)`). The exporter also creates fake `GPS Latitude (deg)` /
`GPS Longitude (deg)` channels from the simulated position trace so MoTeC can
show a map path; the `.ld` conversion writes MoTeC-compatible display-unit bytes
and an M1/pro-enabled log header so i2 treats exported channels as real
quantities.

Edit `trackType` in `src/+lts/+app/run_simulation.m` to switch between:

- `straight10`, `straight`, `straight75` — acceleration / top-speed validation
- `oval`, `skidpad`, `autocross`, `busstop`, `slalom`, `90turn` — built-in courses
- `2026enduro` — loads a `.mat` centerline from `tracks/` (see [Track files](#track-files))

`skidpad` simulates one warmup lap before the timed lap; returned plots and
MoTeC exports contain only the second lap.

## Project objective and evidence policy

The primary output is a transportable prediction of absolute lap time and the
change in lap time caused by a physical vehicle modification. Correlation is
split into component evidence, governed plant calibration, whole-run plant
validation, and minimum-lap-time optimization.

A close replay fit is not by itself evidence that a modified vehicle is
predicted correctly. Production studies accept only a governed parameter
manifest and a non-legacy calibration artifact. Parameters classified as
`design` or `fixed_measured` cannot be fitted. A changed vehicle reuses the
same global calibration and run scenario, and remains `uncertified` until a
known real-car A/B intervention passes the transport-validation gate. The
supplied R25 artifact is intentionally `provisional`: current baseline-only
logs cannot establish counterfactual accuracy.

- **Build the initial model** — see [setup.md](setup.md).
- **Calibrate, validate, run a design study, transport-validate** — see [workflow.md](workflow.md).
- **Governed plant correlation & certification** — see the [Governed Prediction](https://jyjh.github.io/lts/governed-prediction/) page.

### Quick Predict

```matlab
addpath('src')
result = lts.app.predict_design_change( ...
    'Change', 'config/design_changes/example_ballast_removal.json', ...
    'Track', '2026enduro', ...
    'AllowProvisional', true)
```

`AllowProvisional=true` explicitly opts into an uncertified engineering study;
it does not promote the calibration. Normalize a complete run with
`lts.app.preprocess_plant_data`, catalog it by whole-run role, and validate it
with `lts.app.run_plant_validation`.

## Correlation Replay

`lts.app.run_correlation` replays real MoTeC controls through the simulator: it
extracts the measured throttle, brake, steer, and starting speed from a selected
lap (or the whole log when `Lap` is omitted) and uses those controls instead of
`lts.driver.DriverModel`, then exports a new simulated CSV/`.ld` for overlay in
MoTeC i2.

```matlab
addpath('src')
lts.app.run_correlation( ...
    'MoTeCFile', 'data/real_run.ld', ...
    'Lap', 4, ...
    'VehicleConfig', @lts.vehicles.R25, ...
    'TuningFile', 'R25_correlation_tuning', ...
    'Track', '2026enduro')
```

Replay is free-space input replay: the imported controls and initial state drive
the physics, which then produces its own path. There is no GPS/course alignment,
track rebasing, or track-limit stop. The configured track is loaded for export
metadata only. Channel maps, brake/powertrain modes (`throttle` /
`motor_torque_command` / `motor_torque_delivered`), pack-power limiting, the R25
inverter-channel decoding, steering transfer curve, and tuning-overlay options
are documented on the **[Correlation Replay](https://jyjh.github.io/lts/correlation/)** page.

### Legacy ML-assisted tuning

`lts.app.tune_correlation` is retained only to reproduce historical diagnostic
work. It requires `LegacyDiagnostic=true`, and its generated overlays cannot be
used by governed design studies.

```matlab
result = lts.app.tune_correlation( ...
    'LegacyDiagnostic', true, ...
    'MoTeCFile', 'data/lap5_raw.ld', ...
    'HorizonS', [3 6 12], 'MaxHours', 8, 'MaxCandidates', 1200, 'Workers', 8)
```

See the **[Correlation Replay](https://jyjh.github.io/lts/correlation/)** page
for the anchor-split, GPS-kinematics scoring, and checkpoint/resume behavior.

## Track files

`2026enduro` and other real circuits are loaded from `.mat` files in `tracks/`
via `lts.components.WaypointTrack.loadMat`. These files are produced by the separate
[`fsae track image tool`](https://github.com/jyjh/fsae-track-image-tool), which
traces a track image into `[x, y]` waypoints.

**Travel direction.** The exporter bakes the requested clockwise/anticlockwise
direction into the ordering of `points_m` and also records it in a `direction`
field. `loadMat` honors that order by default; an explicit override forces a
direction:

```matlab
track = lts.components.WaypointTrack.loadMat('tracks/<file>.mat', 'Direction', 'anticlockwise');
```

If the override conflicts with the direction stored in the file (for example
because the file is a **stale copy** that was re-exported the other way), the
waypoints are reversed — keeping the start/finish point fixed — and a warning is
emitted. `lts.app.run_simulation` loads the endurance track without a
`Direction` override, so the stored `direction` field is honored as-is, and
prints the resolved direction at startup, so a wrong or stale track is obvious
immediately. A file with no direction field at all also warns.

**Updating a track.** The exporter writes to its own `examples/` directory; it
does **not** touch this repo's `tracks/`. After re-running the exporter (with a
changed `Direction` or otherwise), copy the new `.mat` into `tracks/` yourself.
`Direction` only reorders points *inside* the file — the filename is unchanged —
so a re-export silently overwrites the previous output.

**Variable track widths.** As of the cone-aware exporter, the `.mat`/`.csv`
carry the real track corridor, not a single width: each waypoint has its own
`width_m` plus asymmetric `left_width_m`/`right_width_m` derived from the cone
marks. `WaypointTrack.loadMat` reads these into `LeftWidth`/`RightWidth`, and
the simulator consumes them with full per-side fidelity — the off-track margin,
edge slowdown/steering, racing-line offset, and feasibility all use the actual
local half-width on whichever side of the centerline the car is on (positive
lateral error = left of the line, bounded by `left_width_m`; negative by
`right_width_m`). When a direction override reverses the waypoint order, the
left and right sides are swapped to stay consistent with the new travel
direction. Scalar-width files (no `left_width_m`/`right_width_m`) load exactly
as before and run with a symmetric `Width/2` corridor.

```matlab
track = lts.components.WaypointTrack.loadMat('tracks/<file>.mat');
[leftWidth, rightWidth] = track.getTrackSideWidths();   % per-waypoint [m]
```

The app entry points (`run_simulation`, `run_all`, `CorrelationAppSupport`)
load the file's widths as-is; do not override `track.Width`, since that would
discard the exported corridor and force a uniform width back on.

## Current Model

- **Whole-car aero** — `lts.components.Aero.WholeCarAero` uses a single ClA/CdA and center-of-pressure location from `cfg.aero`.
- **Transient chassis platform** — `lts.components.Chassis.SimpleChassis` tracks heave, pitch, and separate front/rear roll DOFs coupled by chassis torsional rigidity for chassis-driven corner loads. Telemetry includes front/rear roll angles and roll rates.
- **Four-corner transient suspension** — `lts.components.Suspension.SuspensionManager` manages one `SimpleSuspension` and `SuspensionState` per corner.
- **Table-based suspension and steering geometry** — `lts.components.Suspension.SuspensionGeometry` provides camber, toe, motion ratio, steering axis caster/trail/scrub radius/kingpin inclination, and Ackermann steering presets. Positive caster tilts the axis rearward, positive trail places the contact patch behind the kingpin ground point, and positive scrub radius places it outboard.
- **EMRAX 228 powertrain** — `lts.components.Powertrain.EMRAX228Powertrain` loads `EMRAX228LC Single_3.36.mat`, tracks motor RPM with `PowertrainState`, applies a constant-power torque rolloff above the data endpoint, and enforces a hard RPM cap.
- **Pacejka tire model** — `lts.components.Tire.PacejkaTire` loads the provided `.tir` file and tracks per-corner tire state, including suspension-derived camber and per-corner slip angles.
- **Test tracks** — `lts.components.TestTrack` provides straight, oval, skidpad, autocross, busstop, slalom, and 90-turn layouts.
- **MoTeC telemetry export** — `lts.telemetry.TelemetryExporter.exportToMoTeCLog` writes simulation logs as MotecLogGenerator-compatible CSVs and converts them to MoTeC `.ld` files through the MotecLogGenerator submodule.

For the force equations and per-step data flow, see the
[Physics Flow](https://jyjh.github.io/lts/physics-flow/) page.

## Submodules and Python tooling

If `external/MotecLogGenerator` is missing after cloning, initialize it:

```bash
git submodule update --init --recursive
```

The `.ld` conversion uses Python and MotecLogGenerator's dependencies:

```bash
python -m pip install cantools numpy
```

The vehicle generator and analysis scripts under `scripts/` are documented in
[`scripts/README.md`](scripts/README.md).

## Documentation

Full documentation is available at [jyjh.github.io/lts](https://jyjh.github.io/lts).

| Page | Covers |
|---|---|
| [Architecture & Usage](https://jyjh.github.io/lts/) | Component model, simulation loop, key files |
| [Class Diagram](https://jyjh.github.io/lts/class-diagram/) | UML, design patterns, composition |
| [Simulation Loop](https://jyjh.github.io/lts/simulation-loop/) | Per-timestep sequence |
| [Physics Flow](https://jyjh.github.io/lts/physics-flow/) | Force equations, sign conventions, source map |
| [Data Ingestion](https://jyjh.github.io/lts/data-ingestion/) | EMRAX/tire data, MoTeC export, correlation data flow |
| [Correlation Replay](https://jyjh.github.io/lts/correlation/) | Channel maps, brake/powertrain modes, tuning overlays |
| [Governed Prediction](https://jyjh.github.io/lts/governed-prediction/) | Parameter roles, calibration, certification gate |
| [Department Workflow](https://jyjh.github.io/lts/workflow/) | Subsystem-data assembly diagram |

Guides that live at the repo root: [setup.md](setup.md) (initial car model) and
[workflow.md](workflow.md) (car data → design decision).

## Requirements

- MATLAB R2019b or later
- [MFeval](https://www.mathworks.com/matlabcentral/fileexchange/63618-mfeval) for Pacejka Magic Formula tire evaluation
- The provided EMRAX and tire data files in `src/+lts/+components/+Powertrain` and `src/+lts/+components/+Tire`
- Python 3 with `cantools` and `numpy` for MoTeC `.ld` export through the [MotecLogGenerator](https://github.com/jyjh/MotecLogGenerator) submodule

## License

See [LICENSE](LICENSE) for details.
