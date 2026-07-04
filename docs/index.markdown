---
layout: home
title: Home
---

An object-oriented MATLAB lap-time simulation framework for FSAE vehicles.

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
- `EMRAX228Powertrain` uses the provided `EMRAX228CC Single_4.5.mat` tractive-force map, applies configurable torque falloff after the map endpoint, and cuts drive force at the hard RPM cap.
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
- `oval` - oval with straights and constant-radius turns
- `skidpad` - FSAE skidpad circle; one warmup lap is simulated before the recorded lap
- `autocross` - mixed low-speed course
- `busstop` - open chicane layout
- `slalom` - short launch straight into alternating slalom offsets
- `90turn` - open 90-degree turn with straights before and after

Tune the EMRAX powertrain after construction if needed:

```matlab
powertrain = lts.components.Powertrain.EMRAX228Powertrain();
powertrain.rpmFalloffFactor = 2.0;  % steeper torque falloff above the map endpoint
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

## Requirements

- MATLAB R2019b or later
- MFeval for Pacejka tire evaluation
- Provided EMRAX `.mat` and tire `.tir` files
