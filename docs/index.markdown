---
layout: home
title: Home
---

An object-oriented MATLAB simulation for transient-state Lap Time Simulation (LTS) for FSAE vehicles.

## Architecture

> 📊 **[View the UML Class Diagram](class-diagram)** — Full Mermaid.js class diagram showing inheritance, composition, and aggregation relationships.

The simulation uses a **composition pattern** where swappable component objects are assembled by a `VehicleManager` orchestrator:

```
VehicleManager
├── AeroManager              ← Aggregates multiple positioned aero components
│   ├── FrontWing            ← Pitch & height sensitive (ground effect)
│   │     (AeroComponent)      x=+0.9m from CG
│   ├── RearWing             ← Pitch sensitive (AoA changes)
│   │     (AeroComponent)      x=-0.85m from CG
│   └── UnderbodyFloor       ← Extreme height sensitivity + stall model
│         (AeroComponent)      x=0m (at CG)
├── SuspensionComponent      ← Abstract interface
│   └── SimpleSuspension     ← Fixed geometry, roll stiffness split
├── PowertrainComponent      ← Abstract interface
│   └── SimplePowertrain     ← Single-gear with torque curve
├── TireModel                ← Abstract interface
│   └── SimpleTire           ← Linear with saturation + load sensitivity
└── Track                    ← Abstract interface
    └── TestTrack            ← straight / oval / skidpad / autocross
```

### Multi-Component Aero System

Each aero component is an independent `AeroComponent` with:
- **Position** (`xPosition`, `zPosition`) — where it sits on the car
- **Vehicle state awareness** — receives full `VehicleState` including:
  - `pitchAngle` — from longitudinal acceleration (braking = nose up)
  - `rideHeight` — deviation from nominal (for future: track elevation)
- **Pitch & height sensitivity** — each component responds differently:
  - **FrontWing**: loses downforce under braking (nose rises), height-sensitive ground effect
  - **RearWing**: gains downforce under braking (rear squats), moderate sensitivity
  - **UnderbodyFloor**: exponential ground effect model with stall at very low ride heights

The `AeroManager` aggregates all components and computes:
- Total downforce and drag (sum of all components)
- Aero balance via moment calculation from positioned forces
- Pitch angle from longitudinal acceleration (`pitchStiffness × ax`)

### Swapping Components

Each component inherits from an abstract class. To use a different model, create a new class that inherits the same interface:

```matlab
classdef MyCustomFloor < components.AeroComponent
    methods
        function F = computeDownforce(obj, vehicleState)
            % Your pitch/height-aware implementation
            pitchFactor = 1 + mySensitivity * vehicleState.pitchAngle;
            F = 0.5 * obj.rho * obj.ClA * pitchFactor * vehicleState.speed^2;
        end
        function F = computeDrag(obj, vehicleState)
            F = 0.5 * obj.rho * obj.CdA * vehicleState.speed^2;
        end
        function r = getAirDensity(obj)
            r = obj.rho;
        end
    end
end
```

Then register it with AeroManager:

```matlab
aero = components.AeroManager(1.55);  % wheelbase
aero = aero.addComponent(MyCustomFloor('xPosition', 0, 'zPosition', 0.035));
```

## Usage

Run the main script in MATLAB:

```matlab
run_simulation
```

Change the track type by editing the `trackType` variable in `run_simulation.m`:
- `'straight'` — 200m straight for top speed validation
- `'oval'` — Oval with 60m straights and 15m radius turns
- `'skidpad'` — FSAE skidpad (8.125m radius circle)
- `'autocross'` — Mixed corner track with hairpins and chicanes

## File Structure

```
+components/               % MATLAB package namespace
  AeroComponent.m          % Abstract aero interface (positioned, state-aware)
  AeroManager.m            % Aggregates multiple aero components
  FrontWing.m              % Front wing
  RearWing.m               % Rear wing
  UnderbodyFloor.m         % Floor/diffuser with stall model
  SimpleAero.m             % Generic constant-coefficient aero (legacy)
  SuspensionComponent.m    % Abstract suspension interface
  SimpleSuspension.m       % Fixed geometry suspension
  PowertrainComponent.m    % Abstract powertrain interface
  SimplePowertrain.m       % Single-gear with torque curve
  TireModel.m              % Abstract tire interface
  SimpleTire.m             % Linear tire with saturation
  Track.m                  % Abstract track interface
  TestTrack.m              % Test track implementations
VehicleState.m             % Vehicle state (speed, pitch, ride height, etc.)
VehicleManager.m           % Simulation orchestrator
run_simulation.m           % Entry-point script with plotting
```

## Simulation Model

- **Bicycle model** with transient load transfer
- **Euler integration** at 1ms timestep
- **Multi-element aero** with positioned components responding to pitch and ride height
- **Pitch model**: angle derived from longitudinal acceleration via pitch stiffness
- **Driver model** with look-ahead braking based on upcoming curvature
- **Force balance**: drive force, brake force, aerodynamic drag, rolling resistance
- **Tire grip limit** determines maximum cornering speed for each curvature
- **Aero-enhanced grip**: downforce increases tire loads, raising cornering capability

## Extending

| To add | Do this |
|--------|---------|
| Pacejka tires | Create a new `TireModel` with Pacejka formula |
| Dynamic aero map | Create a new `AeroComponent` with ride height lookup table |
| Real track data | Create a new `Track` from GPS/cone waypoints |
| 4-wheel model | Upgrade `VehicleManager` with individual wheel loads |
| Track elevation | Add elevation profile to `Track` and feed into `VehicleState.rideHeight` |
| RK4 integration | Add integration method option to `VehicleManager` |

## Requirements

- MATLAB R2019b or later