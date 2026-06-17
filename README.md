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

## Current Model

- Multi-element aero system: `FrontWing`, `RearWing`, and `UnderbodyFloor` aggregated by `components.Aero.AeroManager`.
- Four-corner transient suspension: `components.Suspension.SuspensionManager` manages one `SimpleSuspension` and `SuspensionState` per corner.
- Table-based suspension and steering geometry: `components.Suspension.SuspensionGeometry` provides camber, toe, motion ratio, and Ackermann steering presets. Switch `geometryPreset` in `src/run_simulation.m` between `neutral`, `baseline`, `high-camber-gain`, and `pro-ackermann`.
- EMRAX 228 powertrain: `components.Powertrain.EMRAX228Powertrain` loads `EMRAX228CC Single_4.5.mat`, tracks motor RPM with `PowertrainState`, applies torque falloff above the data endpoint, and enforces a hard RPM cap.
- Pacejka tire model: `components.Tire.PacejkaTire` loads the provided `.tir` file and tracks per-corner tire state, including suspension-derived camber and per-corner slip angles.
- Test tracks: `components.TestTrack` provides straight, oval, skidpad, autocross, and busstop layouts.
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
