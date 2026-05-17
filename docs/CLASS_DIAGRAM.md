# LTS — Class Diagram

## Architecture Overview

This MATLAB project simulates a lap-time simulation for an FSAE-style vehicle. It uses the **Strategy Pattern** for swappable component models and the **Composite Pattern** for aggregating aerodynamic elements.

### Design Patterns

| Pattern | Where | Purpose |
|---------|-------|---------|
| **Strategy** | `VehicleManager` accepts abstract component interfaces | Swap aero, suspension, powertrain, tire, or track models without changing simulation logic |
| **Composite** | `AeroManager` extends `AeroComponent` and aggregates multiple `AeroComponent` objects | Treat a single aero element or a collection uniformly |

---

## UML Class Diagram

> **Viewing tip:** This renders natively on GitHub. In VS Code, install the "Markdown Preview Mermaid Support" extension or use the built-in preview.
>
> ⚠️ **Maintainer note:** The diagram below is synced from [`class_diagram.mmd`](class_diagram.mmd). Edit that single file, then run `node docs/sync_diagram.js` to update this Markdown.

```mermaid
classDiagram
    direction TB

    class VehicleManager {
        +aero: components.AeroComponent
        +suspension: components.SuspensionComponent
        +powertrain: components.PowertrainComponent
        +tire: components.TireModel
        +track: components.Track
        +state: VehicleState
        +totalMass: double
        +wheelbase: double
        +dt: double
        +simulate() stateLog, lapTime
        -driverModel() throttle, brake
    }

    class VehicleState {
        +s: double
        +speed: double
        +ax: double
        +ay: double
        +heading: double
        +pitchAngle: double
        +rideHeight: double
        +throttle: double
        +brake: double
        +time: double
        +onTrack: logical
        +updateFromDynamics(ax, ay, ds, dt, curvature, heading, mu)
        +toLogStruct() log
    }

    class AeroComponent {
        <<abstract>>
        +name: string
        +xPosition: double
        +zPosition: double
        +computeDownforce(vehicleState)* double
        +computeDrag(vehicleState)* double
        +getAirDensity()* double
        +getLongitudinalPosition() double
        +computeEffectiveHeight(vehicleState) double
    }

    class AeroManager {
        +components: cell~AeroComponent~
        +wheelbase: double
        +pitchStiffness: double
        +addComponent(aeroComp)
        +removeComponent(name)
        +computeAeroBalance(vehicleState) double
        +computePerComponent(vehicleState) results
        -applyPitchToState(vehicleState) stateOut
    }

    class FrontWing {
        +ClA: double
        +CdA: double
        +rho: double
        +pitchSensitivityClA: double
        +heightSensitivity: double
        +referenceHeight: double
        +computeDownforce(vehicleState) double
        +computeDrag(vehicleState) double
    }

    class RearWing {
        +ClA: double
        +CdA: double
        +rho: double
        +pitchSensitivityClA: double
        +heightSensitivity: double
        +referenceHeight: double
        +computeDownforce(vehicleState) double
        +computeDrag(vehicleState) double
    }

    class UnderbodyFloor {
        +ClA: double
        +CdA: double
        +rho: double
        +pitchSensitivityClA: double
        +referenceHeight: double
        +stallHeight: double
        +heightExponent: double
        +computeDownforce(vehicleState) double
        +computeDrag(vehicleState) double
    }

    class SimpleAero {
        +ClA: double
        +CdA: double
        +rho: double
        +pitchSensitivityClA: double
        +pitchSensitivityCdA: double
        +computeDownforce(vehicleState) double
        +computeDrag(vehicleState) double
    }

    class SuspensionComponent {
        <<abstract>>
        +computeLatLoadTransfer(ay, totalMass)* latTransfer
        +computeLongLoadTransfer(ax, totalMass)* longTransfer
        +getRollStiffnessDistribution()* double
        +getStaticWeightDistribution()* double
    }

    class SimpleSuspension {
        +trackWidth: double
        +wheelbase: double
        +cgHeight: double
        +rollStiffDist: double
        +staticFrontWeight: double
        +computeLatLoadTransfer(ay, totalMass) latTransfer
        +computeLongLoadTransfer(ax, totalMass) longTransfer
    }

    class PowertrainComponent {
        <<abstract>>
        +computeDriveForce(speed, throttle)* double
        +getMaxTorque(engineSpeed)* double
        +getTotalGearRatio()* double
        +getDrivetrainEfficiency()* double
    }

    class SimplePowertrain {
        +maxEngineTorque: double
        +totalGearRatio: double
        +wheelRadius: double
        +drivetrainEfficiency: double
        +torqueCurveRPM: double[]
        +torqueCurveNm: double[]
        +computeDriveForce(speed, throttle) double
    }

    class TireModel {
        <<abstract>>
        +computeLateralForce(normalLoad, slipAngle, mu)* double
        +computeLongitudinalForce(normalLoad, slipRatio, mu)* double
        +getPeakFriction(normalLoad)* double
    }

    class SimpleTire {
        +corneringStiffness: double
        +longitudinalStiffness: double
        +peakMuLat: double
        +peakMuLong: double
        +loadSensitivityExp: double
        +computeLateralForce(normalLoad, slipAngle, mu) double
        +getPeakFriction(normalLoad) double
    }

    class Track {
        <<abstract>>
        +getTrackPoints()* double[][]
        +getCurvature()* double[]
        +getSurfaceFriction()* double[]
        +getTotalLength()* double
        +getHeading()* double[]
        +resampleTrack(points, ds)$ double[][]
        +computeCurvature(points)$ double[][]
        +computeHeading(points)$ double[]
    }

    class TestTrack {
        +trackPoints: double[][]
        +trackCurvature: double[]
        +trackHeading: double[]
        +trackLength: double
        +getTrackPoints() double[][]
        +getCurvature() double[]
        +getTotalLength() double
    }

    %% ── Inheritance ──────────────────────────────────────
    AeroComponent <|-- AeroManager
    AeroComponent <|-- FrontWing
    AeroComponent <|-- RearWing
    AeroComponent <|-- UnderbodyFloor
    AeroComponent <|-- SimpleAero

    SuspensionComponent <|-- SimpleSuspension
    PowertrainComponent <|-- SimplePowertrain
    TireModel <|-- SimpleTire
    Track <|-- TestTrack

    %% ── Composition ─────────────────────────────────────
    VehicleManager *-- AeroComponent : aero
    VehicleManager *-- SuspensionComponent : suspension
    VehicleManager *-- PowertrainComponent : powertrain
    VehicleManager *-- TireModel : tire
    VehicleManager *-- Track : track
    VehicleManager *-- VehicleState : state

    %% ── Aggregation (Composite pattern) ─────────────────
    AeroManager o-- AeroComponent : components
```

---

## Relationship Summary

### Inheritance (`extends`)

| Abstract Base | Concrete Implementation |
|--------------|------------------------|
| `components.AeroComponent` | `AeroManager`, `FrontWing`, `RearWing`, `UnderbodyFloor`, `SimpleAero` |
| `components.SuspensionComponent` | `SimpleSuspension` |
| `components.PowertrainComponent` | `SimplePowertrain` |
| `components.TireModel` | `SimpleTire` |
| `components.Track` | `TestTrack` |

### Composition (`owns`)

| Owner | Property | Type | Cardinality |
|-------|----------|------|-------------|
| `VehicleManager` | `aero` | `AeroComponent` | 1 |
| `VehicleManager` | `suspension` | `SuspensionComponent` | 1 |
| `VehicleManager` | `powertrain` | `PowertrainComponent` | 1 |
| `VehicleManager` | `tire` | `TireModel` | 1 |
| `VehicleManager` | `track` | `Track` | 1 |
| `VehicleManager` | `state` | `VehicleState` | 1 |

### Aggregation (`manages`)

| Owner | Property | Type | Cardinality |
|-------|----------|------|-------------|
| `AeroManager` | `components` | `AeroComponent` | 0..* |

### Data Flow

`VehicleManager.simulate()` orchestrates the simulation loop:

1. **Track** → curvature, friction, heading at current position
2. **Driver Model** → throttle/brake decision based on look-ahead
3. **AeroComponent** → downforce, drag (uses `VehicleState` for pitch/ride height)
4. **SuspensionComponent** → load transfer, weight distribution
5. **PowertrainComponent** → drive force from speed & throttle
6. **TireModel** → peak friction from normal load
7. **VehicleState** → integrate dynamics forward one timestep