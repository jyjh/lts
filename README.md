# FSAE Transient Lap Time Simulation

An object-oriented MATLAB lap-time simulation framework for FSAE vehicles. The project composes swappable aero, suspension, powertrain, tire, and track models, then runs a transient simulation loop through `Simulator`.

## Quick Start

```matlab
run_simulation
```

Each run writes a MotecLogGenerator-compatible CSV to `exports/motec_<track>_<timestamp>.csv`.
It also uses the `external/MotecLogGenerator` submodule to create
`exports/motec_<track>_<timestamp>.ld` for MoTeC i2.
Set `exportMoTeC = false` in `src/run_simulation.m` to disable this.

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

Edit `trackType` in `src/run_simulation.m` to switch between:

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
via `components.WaypointTrack.loadMat`. These files are produced by the separate
[`fsae track image tool`](https://github.com/jyjh/fsae-track-image-tool), which
traces a track image into `[x, y]` waypoints.

**Travel direction.** The exporter bakes the requested clockwise/anticlockwise
direction into the ordering of `points_m` and also records it in a `direction`
field. `loadMat` honors that order by default, and an explicit override can be
passed to force a direction:

```matlab
track = components.WaypointTrack.loadMat('tracks/<file>.mat', 'Direction', 'anticlockwise');
```

If the override conflicts with the direction stored in the file (for example
because the file is a **stale copy** that was re-exported the other way), the
waypoints are reversed — keeping the start/finish point fixed — and a warning is
emitted. `run_simulation.m` passes `'Direction', 'anticlockwise'` for the
endurance track and prints the resolved direction at startup, so a wrong or
stale track is obvious immediately. A file with no direction field at all also
warns.

**Updating a track.** The exporter writes to its own `examples/` directory; it
does **not** touch this repo's `tracks/`. After re-running the exporter (with a
changed `Direction` or otherwise), copy the new `.mat` into `tracks/` yourself.
Note that `Direction` only reorders points *inside* the file — the filename is
unchanged — so a re-export silently overwrites the previous output.

## Current Model

- Whole-car aero system: `components.Aero.WholeCarAero` uses a single ClA/CdA and center-of-pressure location from `cfg.aero`.
- Transient chassis platform: `components.Chassis.SimpleChassis` tracks heave, pitch, and roll for chassis-driven corner loads.
- Four-corner transient suspension: `components.Suspension.SuspensionManager` manages one `SimpleSuspension` and `SuspensionState` per corner.
- Table-based suspension and steering geometry: `components.Suspension.SuspensionGeometry` provides camber, toe, motion ratio, and Ackermann steering presets. Switch `geometryPreset` in `src/run_simulation.m` between `neutral`, `baseline`, `high-camber-gain`, and `pro-ackermann`.
- EMRAX 228 powertrain: `components.Powertrain.EMRAX228Powertrain` loads `EMRAX228CC Single_4.5.mat`, tracks motor RPM with `PowertrainState`, applies torque falloff above the data endpoint, and enforces a hard RPM cap.
- Supported Pacejka tire model: `components.Tire.PacejkaTire` loads the provided `.tir` file and tracks per-corner tire state, including suspension-derived camber and per-corner slip angles.
- Test tracks: `components.TestTrack` provides straight, oval, skidpad, autocross, busstop, slalom, and 90-turn layouts.
- MoTeC telemetry export: `TelemetryExporter.exportToMoTeCLog` writes simulation logs as MotecLogGenerator-compatible CSVs and converts them to MoTeC `.ld` files through the MotecLogGenerator submodule.

## Documentation

Full documentation is available at [jyjh.github.io/lts](https://jyjh.github.io/lts).

- [Architecture & Usage](https://jyjh.github.io/lts/)
- [UML Class Diagram](https://jyjh.github.io/lts/class-diagram/)
- [Simulation Loop](https://jyjh.github.io/lts/simulation-loop/)
- [Department Workflow](https://jyjh.github.io/lts/workflow/)
- [Data Ingestion](https://jyjh.github.io/lts/data-ingestion/)

## Requirements

- MATLAB R2019b or later
- [MFeval](https://www.mathworks.com/matlabcentral/fileexchange/63618-mfeval) for Pacejka Magic Formula tire evaluation
- The provided EMRAX and tire data files in `src/+components/+Powertrain` and `src/+components/+Tire`
- Python 3 with `cantools` and `numpy` for MoTeC `.ld` export through the [MotecLogGenerator](https://github.com/stevendaniluk/MotecLogGenerator) submodule

## License

See [LICENSE](LICENSE) for details.
