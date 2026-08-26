---
layout: home
title: Home
---

An object-oriented MATLAB lap-time simulation framework for FSAE vehicles.

## What this project is for

The primary output is a **transportable prediction** of absolute lap time and
the change in lap time caused by a physical vehicle modification. A close replay
fit is not by itself evidence that a modified vehicle is predicted correctly, so
correlation is split into stages, each with its own evidence:

1. **Build the model** from measurements, CAD, and supplier data —
   [`setup.md`](https://github.com/jyjh/lts/blob/main/setup.md) on the repo.
2. **Replay real controls** through the same physics to find model errors —
   [Correlation Replay](/lts/correlation/).
3. **Calibrate** only manifest parameters marked `global_calibrated`, one stage
   at a time — [Governed Prediction](/lts/governed-prediction/).
4. **Validate** against a complete held-out run with a single initialization —
   [`workflow.md`](https://github.com/jyjh/lts/blob/main/workflow.md).
5. **Predict** a provenance-backed design change without refitting, then
   **certify** it only after a known A/B intervention passes the real-car gate.

Parameters classified as `design` or `fixed_measured` cannot be fitted. A
changed vehicle reuses the same global calibration and remains `uncertified`
until the transport-validation gate passes. The supplied R25 artifact is
intentionally `provisional`.

## Architecture

The project uses a composition-based vehicle model. `lts.vehicle.VehicleManager` stores the selected vehicle components and parameters, while `lts.simulation.Simulator` owns the timestep loop. Components remain swappable through abstract interfaces where practical.

```text
lts.vehicle.VehicleManager
|-- lts.components.Aero.WholeCarAero
|-- lts.components.Chassis.SimpleChassis
|   `-- ChassisState
|-- lts.components.Suspension.SuspensionManager
|   |-- SimpleSuspension + SuspensionState (FL)
|   |-- SimpleSuspension + SuspensionState (FR)
|   |-- SimpleSuspension + SuspensionState (RL)
|   `-- SimpleSuspension + SuspensionState (RR)
|-- lts.components.Powertrain.EMRAX228Powertrain
|   `-- PowertrainState
|-- lts.components.Tire.PacejkaTire
|   |-- TireState (FL)
|   |-- TireState (FR)
|   |-- TireState (RL)
|   `-- TireState (RR)
`-- lts.components.TestTrack
```

See the [class diagram](class-diagram/) for a fuller relationship map.
See the [physics flow](physics-flow/) for the force equations and timestep data flow.

## Simulation Model

- `lts.driver.DriverModel` reads the current state and upcoming curvature to choose throttle and brake.
- `WholeCarAero` resolves a single center-of-pressure aero resultant into front/rear downforce and total drag.
- `SimpleChassis` tracks heave, pitch, and roll from accelerations plus aero pitch moments.
- `SuspensionManager` uses chassis corner motion to update transient tire normal loads, with an algebraic load-transfer fallback when no chassis is configured.
- `PowertrainState` tracks driven-wheel speed and motor RPM, so powertrain force is based on current motor speed rather than vehicle speed alone.
- `EMRAX228Powertrain` uses the provided `EMRAX228LC Single_3.36.mat` tractive-force map, applies a constant-power torque rolloff after the map endpoint, and cuts drive force at the hard RPM cap.
- `PacejkaTire` is the supported tire model and computes per-corner tire forces from slip ratio, slip angle, normal load, contact speed, and surface friction.
- `lts.simulation.VehicleState` integrates speed, position, acceleration, heading, yaw rate, pitch, and elapsed time.

## Usage

Run the main script in MATLAB:

```matlab
addpath('src')
lts.app.run_simulation
```

Change the track type by editing `trackType` in `src/+lts/+app/run_simulation.m`:

- `straight10` - 10 m straight for fast export/debug validation
- `straight` - 200 m straight for acceleration and top-speed validation
- `straight75` - 75 m straight for FSAE Acceleration event validation
- `oval` - oval with straights and constant-radius turns
- `skidpad` - FSAE skidpad circle; one warmup lap is simulated before the recorded lap
- `autocross` - mixed low-speed course
- `busstop` - open chicane layout
- `slalom` - short launch straight into alternating slalom offsets
- `90turn` - open 90-degree turn with straights before and after

Tune the EMRAX powertrain after construction if needed:

```matlab
powertrain = lts.components.Powertrain.EMRAX228Powertrain();
powertrain.rpmLimitRPM = 6500;  % hard motor RPM cap
```

## Key Files

```text
src/+lts/+app/run_simulation.m               Entry-point function
src/+lts/+simulation/Simulator.m             Simulation loop and telemetry logging
src/+lts/+driver/DriverModel.m               Look-ahead driver inputs
src/+lts/+vehicle/VehicleManager.m           Component and vehicle-parameter container
src/+lts/+simulation/VehicleState.m          Vehicle dynamic state
src/+lts/+components/+Aero/                  Aero components and manager
src/+lts/+components/+Suspension/            Four-corner transient suspension
src/+lts/+components/+Powertrain/            EMRAX powertrain and differentials
src/+lts/+components/+Tire/                  Pacejka tire model
src/+lts/+components/TestTrack.m             Built-in test tracks
src/+lts/+telemetry/GraphPlotter.m           Simulation dashboards
```

## Documentation

| Page | Covers |
|---|---|
| [Repositories & Sync](repos/) | The repository family, who works where, how main stays in sync with the components |
| [Department Workflow](workflow/) | Subsystem-data assembly diagram |
| [Tracks](tracks/) | Track `.mat` files, direction handling, variable widths |
| [Class Diagram](class-diagram/) | UML, design patterns, composition |
| [Simulation Loop](simulation-loop/) | Per-timestep sequence |
| [Physics Flow](physics-flow/) | Force equations, sign conventions, source map |
| [Data Ingestion](data-ingestion/) | EMRAX/tire data, MoTeC export, correlation data flow |
| [Correlation Replay](correlation/) | Channel maps, brake/powertrain modes, tuning overlays |
| [Governed Prediction](governed-prediction/) | Parameter roles, calibration, certification gate |
| [Repository Split Plan](repo-split/) | Why the repositories were split; contracts; decision log |
| [Component Contracts](contracts/) | Per-component cfg schemas, telemetry field pins, contract-change process |

Contributing (no git experience needed):
[`CONTRIBUTING.md`](https://github.com/jyjh/lts/blob/main/CONTRIBUTING.md).
Guides on the repo: [`setup.md`](https://github.com/jyjh/lts/blob/main/setup.md)
(initial car model) and [`workflow.md`](https://github.com/jyjh/lts/blob/main/workflow.md)
(car data → design decision).

## Requirements

- MATLAB R2019b or later
- MFeval for Pacejka tire evaluation
- Provided EMRAX `.mat` and tire `.tir` files
