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
|-- components.Chassis.SimpleChassis
|   `-- ChassisState
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

- `DriverModel` builds a local racing speed envelope, commands late braking/full throttle, and shapes steering through active corner segments with yaw-rate and sideslip feedback.
- `AeroManager` resolves positioned aero elements into front/rear downforce and total drag.
- `SimpleChassis` integrates heave, pitch, and roll from longitudinal/lateral acceleration and aero load, then exposes per-corner chassis displacement and velocity.
- `SuspensionManager` uses chassis corner kinematics when available, so spring and compression/rebound damper forces create transient tire normal loads. Without a chassis component, it falls back to the older algebraic load-transfer path.
- `PowertrainState` tracks driven-wheel speed and motor RPM, so powertrain force is based on current motor speed rather than vehicle speed alone.
- `EMRAX228Powertrain` uses the provided `EMRAX228CC Single_4.5.mat` tractive-force map, applies configurable torque falloff after the map endpoint, and cuts drive force at the hard RPM cap.
- `PacejkaTire` computes per-corner combined-slip tire forces from local wheel-plane speed, slip ratio, slip angle, normal load, and surface friction, with simple Ackermann front steering and relaxation lengths for transient force buildup.
- `VehicleState` integrates speed, position, acceleration, heading, yaw rate, yaw acceleration, lateral velocity, sideslip, chassis pitch/roll/ride height, and elapsed time.

## Usage

Run the main script in MATLAB:

```matlab
run_simulation
```

Change the track type by editing `trackType` in `src/run_simulation.m`:

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
src/+components/+Chassis/                    Heave/pitch/roll chassis platform
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
