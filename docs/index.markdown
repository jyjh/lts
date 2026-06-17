---
layout: home
title: Home
---

An object-oriented MATLAB lap-time simulation framework for FSAE vehicles.

## Architecture

The project uses a composition-based vehicle model. `VehicleManager` stores the selected vehicle components and parameters, while `Simulator` owns the timestep loop. Components remain swappable through abstract interfaces where practical.

```text
VehicleManager
|-- components.Aero.AeroManager
|   |-- FrontWing
|   |-- RearWing
|   `-- UnderbodyFloor
|-- components.Suspension.SuspensionManager
|   |-- SimpleSuspension + SuspensionState (FL)
|   |-- SimpleSuspension + SuspensionState (FR)
|   |-- SimpleSuspension + SuspensionState (RL)
|   `-- SimpleSuspension + SuspensionState (RR)
|-- components.Powertrain.EMRAX228Powertrain
|   `-- PowertrainState
|-- components.Tire.PacejkaTire
|   |-- TireState (FL)
|   |-- TireState (FR)
|   |-- TireState (RL)
|   `-- TireState (RR)
`-- components.TestTrack
```

See the [class diagram](class-diagram/) for a fuller relationship map.

## Simulation Model

- `DriverModel` reads the current state and upcoming curvature to choose throttle and brake.
- `AeroManager` resolves positioned aero elements into front/rear downforce and total drag.
- `SuspensionManager` combines static load, aero load, lateral load transfer, and longitudinal load transfer, then updates each corner's transient suspension state.
- `PowertrainState` tracks driven-wheel speed and motor RPM, so powertrain force is based on current motor speed rather than vehicle speed alone.
- `EMRAX228Powertrain` uses the provided `EMRAX228CC Single_4.5.mat` tractive-force map, applies configurable torque falloff after the map endpoint, and cuts drive force at the hard RPM cap.
- `PacejkaTire` computes per-corner tire forces from slip ratio, slip angle, normal load, and surface friction.
- `VehicleState` integrates speed, position, acceleration, heading, yaw rate, pitch, and elapsed time.

## Usage

Run the main script in MATLAB:

```matlab
run_simulation
```

Change the track type by editing `trackType` in `src/run_simulation.m`:

- `straight10` - 10 m straight for fast export/debug validation
- `straight` - 200 m straight for acceleration and top-speed validation
- `oval` - oval with straights and constant-radius turns
- `skidpad` - FSAE skidpad circle
- `autocross` - mixed low-speed course
- `busstop` - open chicane layout

Tune the EMRAX powertrain after construction if needed:

```matlab
powertrain = components.Powertrain.EMRAX228Powertrain();
powertrain.rpmFalloffFactor = 2.0;  % steeper torque falloff above the map endpoint
```

## Key Files

```text
src/run_simulation.m                         Entry-point script
src/Simulator.m                              Simulation loop and telemetry logging
src/DriverModel.m                            Look-ahead driver inputs
src/VehicleManager.m                         Component and vehicle-parameter container
src/VehicleState.m                           Vehicle dynamic state
src/+components/+Aero/                       Aero components and manager
src/+components/+Suspension/                 Four-corner transient suspension
src/+components/+Powertrain/                 Simple and EMRAX powertrains
src/+components/+Tire/                       Simple and Pacejka tire models
src/+components/TestTrack.m                  Built-in test tracks
src/GraphPlotter.m                           Simulation dashboards
```

## Requirements

- MATLAB R2019b or later
- MFeval for Pacejka tire evaluation
- Provided EMRAX `.mat` and tire `.tir` files
