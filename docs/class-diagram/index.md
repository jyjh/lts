---
layout: page
title: Class Diagram
permalink: /class-diagram/
---

## Architecture Overview

The simulation separates vehicle configuration from simulation execution:

- `VehicleManager` stores component references and vehicle-level constants.
- `Simulator` runs each timestep, asks `DriverModel` for inputs, computes forces, updates subsystem state, integrates `VehicleState`, and logs telemetry.
- Component classes live under the MATLAB `components` package and can be swapped when they satisfy the relevant interface.

## Design Patterns

| Pattern | Where | Purpose |
|---------|-------|---------|
| Strategy | `VehicleManager` accepts powertrain, tire, aero, suspension, and track component objects | Swap subsystem models without changing the simulation loop |
| Composite | `AeroManager` aggregates multiple `AeroComponent` objects | Resolve several positioned aero elements as one aero system |
| State object | `VehicleState`, `SuspensionState`, `TireState`, `PowertrainState` | Persist transient quantities across timesteps |

---

## UML Class Diagram

> Maintainer note: The diagram below is generated from [`class_diagram.mmd`](class_diagram.mmd). Edit that file, then run `node docs/sync_diagram.js` to regenerate the SVG.

![UML Class Diagram](class_diagram.svg)

---

## Relationship Summary

### Inheritance

| Abstract Base | Concrete Implementations |
|---------------|--------------------------|
| `components.Aero.AeroComponent` | `AeroManager`, `FrontWing`, `RearWing`, `UnderbodyFloor`, `SimpleAero` |
| `components.Suspension.SuspensionComponent` | `SuspensionManager` |
| `components.Powertrain.PowertrainComponent` | `EMRAX228Powertrain`, `SimplePowertrain` |
| `components.Chassis.ChassisComponent` | `SimpleChassis` |
| `components.Tire.TireModel` | `PacejkaTire`; `SimpleTire` is deprecated |
| `components.Track` | `TestTrack` |

### Composition

| Owner | Property | Type |
|-------|----------|------|
| `Simulator` | `vehicleManager` | `VehicleManager` |
| `Simulator` | `driverModel` | `DriverModel` |
| `VehicleManager` | `aero` | `AeroManager` |
| `VehicleManager` | `chassis` | `SimpleChassis` |
| `VehicleManager` | `suspension` | `SuspensionManager` |
| `VehicleManager` | `powertrain` | `PowertrainComponent` |
| `VehicleManager` | `tire` | `TireModel` |
| `VehicleManager` | `track` | `Track` |
| `SuspensionManager` | corner suspensions | `SimpleSuspension` |
| `SimpleSuspension` | `state` | `SuspensionState` |
| `PacejkaTire` | corner states | `TireState` |
| `EMRAX228Powertrain` / `SimplePowertrain` | `state` | `PowertrainState` |

### Data Flow

`Simulator.simulate()` orchestrates the loop:

1. Read current `VehicleState` and track curvature/friction/heading.
2. Ask `DriverModel` for throttle and brake.
3. Compute aero downforce and drag through `AeroManager`.
4. Update chassis heave/pitch/roll and compute chassis-driven corner loads through `SuspensionManager`.
5. Update `PowertrainState` from driven-wheel angular velocity and compute drive force from motor RPM.
6. Solve wheel/contact speed and tire forces through the supported `PacejkaTire` model.
7. Resolve longitudinal and lateral acceleration limits.
8. Integrate `VehicleState`.
9. Append telemetry to `stateLog`.
