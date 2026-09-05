---
layout: page
title: Component Contracts
permalink: /contracts/
---

## The contract, in one page

Every component repository (aero, suspension, powertrain, chassis) has a
written contract with this main repository. It has three parts — the
same three parts as the [Repository Split Plan](../repo-split/):

1. **Interface** — the abstract base class and the methods/properties
   the Simulator calls.
2. **Config schema** — the `cfg.<component>` struct fields the main
   repository passes in, validated by each component's
   `validateConfig` (package function at the repository root).
3. **Telemetry producer fields** — the state-struct property names
   whose values end up in the per-corner and attitude telemetry
   channels.

Parts 2 and 3 are **enforced by CI from both sides**:

- *Producer side* — each component repository's
  `tests/ConformanceTest.m` pins its cfg schema, interface, and
  state-field names. A rename fails that repository's own CI.
- *Consumer side* — the main repository's
  `tests/TelemetryChannelConformanceTest.m` pins the mapping from those
  state fields to `stateLog` channel names (`damperPos_FL` ←
  `SuspensionState.damperPosition`, ...). A rename on either side of
  the boundary fails CI; neither can drift alone.

## Config schemas

All values SI. `validateConfig` throws typed errors
(`lts_<component>_validateConfig:<Case>`) naming the offending field;
the main repository calls every validator in
`lts.vehicle.VehicleManager.fromConfig` before construction.

### Aero (`cfg.aero`)

| Field | Unit | Valid range | Default |
|---|---|---|---|
| `xPosition` | m | finite | −0.084146 |
| `zPosition` | m | ≥ 0 | 0.30 |
| `ClA` | m² | ≥ 0 | 4.10 |
| `CdA` | m² | ≥ 0 | 1.60 |
| `pitchSensitivityClA` | 1/rad | finite (optional) | 0 |

Optional device split — `cfg.aero.components` (all four substructs
required together; `VehicleManager.fromConfig` then builds an
`AeroManager` via `lts.components.Aero.buildFromConfig` instead of the
`WholeCarAero` resultant, adding pitch + ride-height response):

| Substruct | Fields |
|---|---|
| `frontWing` | `xPosition`, `zPosition`, `ClA`, `CdA`, `pitchSensitivityClA`, `heightSensitivity` [1/cm, ≥ 0] |
| `rearWing` | same fields as `frontWing` |
| `floor` | like `frontWing` but `stallHeight` [m, > 0] and `heightExponent` [(0, 2]] instead of `heightSensitivity` |
| `body` | `xPosition`, `zPosition`, `ClA`, `CdA` only (pitch/height-insensitive residual) |

The split must reproduce the whole-car datum exactly at the nominal
attitude: `sum(ClA) == ClA`, `sum(CdA) == CdA`, and the device
downforce center of pressure must equal `xPosition`
(`lts_aero_buildFromConfig:ComponentMismatch` otherwise). Car files
derive the `body` residual from the chosen device shares.

### Chassis (`cfg.chassis`)

| Field | Unit | Valid range | Default |
|---|---|---|---|
| `heaveStiffness` / `heaveDamping` | N/m, N·s/m | ≥ 0 | 160000 / 12000 |
| `pitchStiffness` / `pitchDamping` | N·m/rad, N·m·s/rad | ≥ 0 | 90000 / 6000 |
| `rollStiffness` / `rollDamping` | N·m/rad, N·m·s/rad | ≥ 0 | 55000 / 5000 |
| `torsionalRigidity` | N·m/rad | ≥ 0 or +Inf (exact rigid) | 229183 |
| `torsionalDamping` | N·m·s/rad | ≥ 0 | 2000 |

### Suspension (`cfg.suspension`)

| Field | Unit | Valid range |
|---|---|---|
| `front`/`rear.springRate` | N/m | > 0 |
| `front`/`rear.dampingCoeff`, `.reboundCoeff` | N·s/m | ≥ 0 |
| `motionRatio` | — | (0, 1] |
| `bumpStopLength` / `bumpStopRate` | m, N/m | ≥ 0 |
| `tireSpringRate` | N/m | > 0 |
| `dampingKneeSpeed` | m/s | optional, finite or Inf |
| `dampingHighSpeedRatio` | — | optional, finite |
| `dampingReboundKneeSpeed` | m/s | optional, NaN = none |
| `frontArb`/`rearArb.stiffness` | N·m/rad | ≥ 0 |
| `frontArb`/`rearArb.motionRatio` | — | (0, 1] |
| `frontArb`/`rearArb.leverArm` | m | ≥ 0 |
| `frontArb`/`rearArb.enabled` | — | logical |
| `rollStiffnessOverride` | — | NaN (derive) or [0, 1] |
| `coupleChassisRollToLoadTransfer` | — | logical |
| `geometry` | — | struct with `front`/`rear` (travelGrid + camber/toe/motion-ratio curves, roll-center height) and `steering` (steeringRatio, ackermann, maxWheelSteerAngle) |

### Powertrain (`cfg.powertrain`)

| Field | Unit | Valid range |
|---|---|---|
| `matFile` | path | char/string; `''` = default EMRAX map in the component's `data/powertrain/` |
| `efficiency` | — | (0, 1] |
| `differential.type` | — | `open` \| `locked` \| `lsd` \| `drexler` |
| `finalDriveRatio` | — | optional, NaN (map) or > 0 |
| `motorRotorInertia` | kg·m² | optional, ≥ 0 |
| `efficiencyRpm` / `efficiencyValues` | rpm, — | optional, equal-length vectors |
| `regenEfficiency`, `deliveredTorqueDrivetrainEfficiency` | — | optional, (0, 1] or NaN |
| `regenEnabled` | — | optional, logical |
| `motoringDragTorque` | N·m | optional, ≥ 0 |
| `motoringDragThrottleThreshold` | — | optional, [0, 1] or Inf |
| `regenTorqueLimitNm` | N·m | optional, > 0 |
| `regenEnabledSpeedFloor` | m/s | optional, ≥ 0 |
| `throttleDeadband` | — | optional, [0, 1] |
| `throttleMapInput` / `throttleMapOutput` | — | optional, equal-length vectors |

## Telemetry producer fields

The state-field → channel-name mapping (per corner `FL/FR/RL/RR`):

| Component | State field | Channel |
|---|---|---|
| Suspension | `tireNormalForce` | `Fz_<corner>` |
| Suspension | `suspensionForce` | `suspensionForce_<corner>` |
| Suspension | `antiRollBarForce` | `antiRollBarForce_<corner>` |
| Suspension | `demandedLoad` | `suspensionDemand_<corner>` |
| Suspension | `tireDeflection` | `tireDeflection_<corner>` |
| Suspension | `damperPosition` | `damperPos_<corner>` |
| Suspension | `damperVelocity` | `damperVel_<corner>` |
| Suspension | `sprungPosition`/`sprungVelocity` | same name `_<corner>` |
| Suspension | `unsprungPosition`/`unsprungVelocity` | same name `_<corner>` |
| Suspension | `wheelTravel` | `wheelTravel_<corner>` |
| Suspension | `camberAngle` | `camber_<corner>` |
| Suspension | `toeAngle` | `toe_<corner>` |
| Suspension | `steerAngle` | `wheelSteer_<corner>` |
| Tire | `slipAngle`/`slipRatio`/`peakMu` | same name `_<corner>` |
| Tire | `angularVelocity` | `omega_<corner>` |
| Tire | `Fx` / `Fy` | `tireFx_<corner>` / `tireFy_<corner>` |
| Tire | `relaxedNormalLoad` | `relaxedFz_<corner>` |
| Chassis | `pitchAngle` (via `getPitchAngle`) | `pitchAngle` |
| Chassis | `rollAngle`, front/rear roll angles & rates, `heave` | `rollAngle`, `frontRollAngle`, ..., `rideHeight` |
| Powertrain | `motorRPM`, `motorTorque`, `wheelTorque`, ... | same names |
| Aero | `computeForces` struct (`Fz_front`, `Fz_rear`, `F_drag`, ...) | `aeroFz_front`, `aeroFz_rear`, `F_downforce`, `F_drag` |

## Changing the contract

Contract changes (renaming or re-typing any pinned cfg field, interface
method, or state field above — including the telemetry channel names)
are **two-sided by construction** and require integration-lead
approval. The procedure:

1. **Component PR** (into the component's `staging`): make the change,
   update the component's `tests/ConformanceTest.m` and its README
   contract section in the same PR. Its CI now fails until the
   main-repository side lands — that is intentional; note in the PR
   description that a paired main PR follows.
2. **Main PR** (into `lts@staging`): update the consumer
   (`StateLogBuilder` / `TelemetryExporter` / `VehicleManager` /
   `Simulator` as needed), `tests/TelemetryChannelConformanceTest.m`,
   and this page. Include the component pin bump in the same PR
   (`git submodule update --remote src/...`) so CI proves the new
   combination together.
3. **Golden lap time**: if the contract change shifts the golden
   baseline, that is the subject of the PR discussion — say why, do not
   paper over it.
4. Merge the component PR, then the main PR. The release cascade
   promotes the pair to `main` everywhere, as usual.

Additive changes (a new optional cfg field, a new telemetry channel)
are not contract *breaks*: they still start with a component PR and a
paired main PR, but nothing existing may be renamed or re-typed.

**Rule of thumb for department members:** if your change touches any
name listed on this page, treat it as a contract change and ping the
integration lead before writing code — a five-minute conversation beats
a red CI two repositories later.
