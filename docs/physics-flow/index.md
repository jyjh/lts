---
layout: page
title: Physics Flow
permalink: /physics-flow/
---

## Purpose

This page documents the full simulation path: how a vehicle configuration becomes swappable component objects, how a driver or replay log produces commands, how each physics subsystem computes forces, and how those forces advance `VehicleState`.

The most important design rule is that the car is integrated as a free planar rigid body. The track centerline is used for reference progress, surface mu, driver planning, and lateral-error telemetry. It does not kinematically force the car to follow the centerline.

## Coordinate And Sign Conventions

| Quantity | Convention |
|---|---|
| Body x | Forward |
| Body y | Left |
| Body z | Up for geometry; many suspension states use positive down/compression |
| Yaw | Positive left turn |
| Lateral acceleration `ay` | Positive left |
| Pitch angle | Positive nose up |
| Roll angle | Positive right side down |
| Aero downforce | Stored as positive normal-load contribution |
| Tire `Fx` | Positive driving force in the tire frame |
| Tire `Fy` | Positive left force in the tire frame |
| Slip ratio `kappa` | Positive driving, negative braking |
| Slip angle `alpha` | Positive when the tire must generate leftward force |

## Source Map

| Area | Main files | Role |
|---|---|---|
| Run scripts | `src/run_simulation.m`, `src/run_correlation.m`, `src/run_all.m` | Choose car/track/scenario, build the simulator, export outputs |
| Vehicle setup | `src/VehicleConfig.m`, `src/VehicleManager.m`, `src/+vehicles/*.m` | Store physical parameters, construct components, warm up suspension/chassis |
| State | `src/VehicleState.m` | Hold planar state, attitude telemetry, controls, and reference projection |
| Simulation loop | `src/Simulator.m` | Own timestep loop, force assembly, wheel-contact solve, state integration, replay policies, telemetry |
| Driver | `src/DriverInputPlanner.m`, `src/DriverModel.m` | Build speed/pedal/steer plan and add closed-loop path corrections |
| Replay/correlation | `src/TelemetryReplayDriver.m`, `src/CorrelationReplayProfile.m`, `src/CorrelationStateInitializer.m`, `src/CorrelationTrackAlignment.m` | Normalize logged controls and initial state, then replay them through the same physics |
| Track geometry | `src/+components/Track.m`, `src/+components/WaypointTrack.m`, `src/+components/TestTrack.m` | Provide centerline points, curvature, heading, width, mu, procedural tracks |
| Aero | `src/+components/+Aero/*.m` | Compute quadratic downforce and drag, resolve aero loads to axles |
| Chassis | `src/+components/+Chassis/*.m` | Integrate sprung-mass heave, pitch, roll, and torsional twist |
| Suspension | `src/+components/+Suspension/*.m` | Convert chassis motion/load-transfer demand into per-corner tire normal loads and kinematics |
| Tires | `src/+components/+Tire/*.m` | Compute slip, wheel rotation, Magic Formula forces, relaxation lag, and peak mu |
| Powertrain | `src/+components/+Powertrain/*.m` | Compute motor/wheel torque, motor RPM, coastdown/regen, and differential torque split |
| Telemetry and plots | `src/TelemetryExporter.m`, `src/GraphPlotter.m` | Export MoTeC-compatible logs and visualize state channels |

## Top-Level Flow

`run_simulation.m` is the normal synthetic-lap entry point.

1. Select a `trackType` and vehicle config such as `vehicles.R25()`.
2. Build a `components.TestTrack` or load a `components.WaypointTrack`.
3. Call `VehicleManager.fromConfig(config, track, dt)`.
4. Build `DriverModel(vehicle)` and `Simulator(vehicle, driver, dt)`.
5. Create an initial `VehicleState`.
6. Run `simulator.simulate(initialState, track)`.
7. Export MoTeC CSV/LD and optionally plot telemetry.

`run_correlation.m` follows the same physics path after replacing the driver with measured controls:

1. Extract or read a normalized replay CSV.
2. Build `CorrelationReplayProfile`.
3. Build the vehicle and a measured initial `VehicleState`.
4. Run `Simulator.simulateReplay(...)`.
5. `TelemetryReplayDriver` samples logged throttle, brake, brake pressure, and steer by time or distance.

## Vehicle Construction

`VehicleConfig` is a value object containing all tunable car physics parameters: mass, geometry, aero map, suspension tables, chassis stiffness, powertrain map, differential type, tire data, and brake calibration.

`VehicleManager.fromConfig` turns that configuration into live component objects:

1. `WholeCarAero` from `cfg.aero`.
2. `EMRAX228Powertrain` from the EMRAX `.mat` map and drivetrain settings.
3. `PacejkaTire` from the `.tir` file, with one `TireState` per corner.
4. `VehicleManager` itself, which stores global constants such as mass, wheelbase, CG height, brake bias, and gravity.
5. `SuspensionGeometry`, anti-roll bars, and `SuspensionManager`.
6. Suspension warmup to static tire loads.
7. `SimpleChassis`, settled at static equilibrium and linked back to suspension.
8. The selected differential, usually open, locked, clutch LSD, or Drexler ramp-plate LSD.

This construction order matters because suspension geometry needs vehicle dimensions, chassis roll needs suspension roll stiffness, and powertrain torque splitting needs the differential after the rest of the vehicle exists.

## Driver And Control Planning

The driver is not part of the vehicle physics. It decides requested inputs.

`DriverInputPlanner.buildOpenLoopProfile` creates a distance-indexed plan:

1. It computes a racing-line offset where enabled.
2. It estimates lateral speed limits from curvature:

   ```text
   v_limit = sqrt(a_y_limit / abs(curvature))
   ```

3. It performs a backward braking sweep so each point can slow enough for the next corner.
4. It performs a forward acceleration sweep using powertrain and rear-axle traction capability.
5. It maps desired longitudinal acceleration to pedals:

   ```text
   F_required = mass * ax_ref + F_drag + F_rolling
   throttle = F_required / F_full_throttle
   brake = decel_beyond_coast / brake_decel_per_command
   ```

6. It applies a traction-circle cap, reducing throttle/brake as lateral demand uses more tire grip.

`DriverModel.computeInput` samples that plan and adds closed-loop corrections:

- Stanley-style cross-track correction.
- Heading correction.
- Track-edge steering and slowing.
- Pedal and steering slew limits.
- Optional drive-slip throttle reduction, which behaves like simple traction control by reducing requested throttle before the physics step.

## One `Simulator.step`

`Simulator.step(state, input, ref)` is the core physics transition. It returns a new `VehicleState` plus a force/telemetry struct.

### 1. Normalize Driver Inputs

Throttle and brake are clamped to `[0, 1]`. Normal synthetic runs enforce mutually exclusive pedals and steering slew. Replay runs can disable those policies so logged controls pass through as recorded.

### 2. Compute Aero

`WholeCarAero` uses the standard dynamic-pressure equations:

```text
q = 0.5 * rho * V^2
F_downforce = q * ClA_effective
F_drag = q * CdA
```

`AeroComponent.computeForces` resolves downforce to front and rear axles by moment balance. If `x_cp` is the center of pressure relative to CG and `b` is CG-to-rear axle:

```text
front_fraction = (b + x_cp) / wheelbase
Fz_front_aero = F_downforce * front_fraction
Fz_rear_aero = F_downforce * (1 - front_fraction)
```

Drag is applied later opposite the actual body velocity vector, including sideslip.

### 3. Compute Tire Normal Loads

With a chassis attached, `Simulator` asks:

```text
SuspensionManager.computeCornerLoadsFromChassis(chassis, steer, dt)
```

The chassis state from the previous completed step gives heave, pitch, and front/rear roll. `ChassisState.updateCornerKinematics` converts those attitudes into four suspension pickup displacements and velocities. Each `SimpleSuspension` corner then advances the unsprung mass against the suspension spring/damper/bump stop and tire spring.

Without a chassis, the algebraic fallback computes load transfer directly:

```text
W = mass * g
long_transfer = mass * ax * cg_height / wheelbase
lat_transfer_total = mass * abs(ay) * cg_height / track_width
```

Lateral transfer is split into:

- geometric transfer through front/rear roll centers,
- elastic transfer distributed by spring plus anti-roll-bar roll stiffness.

Anti-roll-bar wheel rate is:

```text
K_w_bar = stiffness * motionRatio^2 / leverArm^2
```

### 4. Update Powertrain State And Torque

The motor speed comes from the driven rear wheels through the differential carrier:

```text
omega_carrier = mean(omega_RL, omega_RR)
motor_rpm = omega_carrier * gear_ratio * 60 / (2*pi)
```

`EMRAX228Powertrain.computeDriveTorque` looks up full-throttle tractive force by motor RPM, scales it by the shaped throttle request and drivetrain efficiency, and returns total rear-axle wheel torque:

```text
F_drive_request = F_map(motor_rpm) * throttle_request * efficiency
T_wheel_total = F_drive_request * wheel_radius
```

At or above the RPM limit, positive drive torque is cut. Optional coastdown and regen create signed rear-axle torque during off-throttle operation.

### 5. Split Torque Through The Differential

`Simulator.solveDifferential` returns per-wheel rear torque and carrier speed.

- `OpenDifferential`: 50/50 torque split, carrier speed is mean wheel speed.
- `LockedDifferential`: 50/50 torque split, then the simulator overwrites both rear wheel speeds to the common carrier speed.
- `ClutchLSDDifferential`: starts at 50/50 and shifts torque toward the slower wheel using preload, ramp torque, and speed-sensitive locking, capped by a bias ratio.
- `DrexlerRampPlateDifferential`: uses signed accel/decel ramp angles and calibrated preload/ramp scale to create a torque-difference capacity.

### 6. Convert Brakes To Wheel Torque

Ratio brake mode computes:

```text
F_brake_capacity = brakeForceCoefficient * totalNormalLoad
F_brake_front = brakeCommand * F_brake_capacity * brakeBiasFront
F_brake_rear = brakeCommand * F_brake_capacity * (1 - brakeBiasFront)
```

Pressure brake mode, used for correlation, converts front/rear line pressure:

```text
F_brake_front = pressure_front_bar * frontForcePerBar
F_brake_rear = pressure_rear_bar * rearForcePerBar
```

Both modes then become wheel torques:

```text
T_brake_front_corner = F_brake_front * R / 2
T_brake_rear_corner = F_brake_rear * R / 2
```

Brake torque is not directly clipped by grip. It changes wheel slip; the tire model determines how much contact-patch braking force is actually generated.

### 7. Solve Wheel Rotation And Tire Slip

Each wheel integrates angular velocity:

```text
I * domega/dt = T_drive - sign*T_brake - Fx*R - T_resist
```

Resistance torque is:

```text
T_resist = sign(omega) * (Crr * Fz * R + C_bearing * abs(omega))
```

The simulator repeats a small wheel-contact solve. Each iteration:

1. advances wheel omega using the latest tire `Fx`,
2. re-solves differential torque at the new rear wheel speeds,
3. updates powertrain motor speed from the carrier,
4. recomputes tire slip and forces.

This reduces the one-step lag between wheel speed and tire force.

Longitudinal slip ratio is:

```text
kappa = (omega*R - Vx_tire) / max(abs(omega*R), abs(Vx_tire), 1.0)
```

At very low speed, the slip ratio blends toward the previous value to avoid numerical sign flips when both wheel and ground speeds are near zero.

### 8. Compute Tire Forces

For each corner, contact patch velocity is CG velocity plus yaw-rate contribution:

```text
vx_corner = vx - yawRate * y_corner
vy_corner = vy + yawRate * x_corner
```

The wheel frame is rotated by road-wheel steer plus toe:

```text
longSpeed = vx_corner*cos(wheelHeading) + vy_corner*sin(wheelHeading)
latSpeed  = -vx_corner*sin(wheelHeading) + vy_corner*cos(wheelHeading)
alpha = atan2(-latSpeed, max(abs(longSpeed), 0.1))
```

`PacejkaTire` applies first-order relaxation so force-producing slip lags kinematic slip:

```text
sigma * d(alpha)/dt + V * alpha = V * alpha_steady
alpha = alpha_steady - (alpha_steady - alpha_previous) * exp(-V*dt/sigma)
```

The same form is applied to `kappa`. `sigma` is `relaxationLength`.

Forces are evaluated through MFeval combined-slip mode:

```text
inputsMF = [Fz, kappa, alpha, camber, phit, Vx, pressure]
```

The raw `.tir` file is treated as the reference dry surface. Track mu scales forces relative to `surfaceMuReference`.

### 9. Sum Body Forces And Yaw Moment

Tire forces are rotated from each wheel frame into the body frame:

```text
Fx_body = Fx_tire*cos(heading) - Fy_tire*sin(heading)
Fy_body = Fx_tire*sin(heading) + Fy_tire*cos(heading)
```

Yaw moment about the CG is:

```text
Mz = x_corner * Fy_body - y_corner * Fx_body
```

Aerodynamic drag opposes the current velocity vector:

```text
netFx = sum(Fx_body) - F_drag * vx / speed
netFy = sum(Fy_body) - F_drag * vy / speed
ax = netFx / mass
ay = netFy / mass
yawAccel = Mz / yawInertia
ay_front = ay + yawAccel * frontArm
ay_rear = ay - yawAccel * rearArm
```

Rolling resistance is already included as a wheel torque and therefore appears through tire `Fx`. It is logged separately but not applied again as a body force.

### 10. Integrate Planar State

`Simulator` integrates yaw rate, yaw angle, world velocity, body velocity, and position. It uses a midpoint yaw for body/world transforms so acceleration projection and velocity reprojection use the same orientation over the step.

`VehicleState.updateFromPlanarDynamics` commits:

- `vx`, `vy`, `speed`,
- `yawRate`, `yaw`, `yawAccel`,
- `x`, `y`,
- `ax`, `ay`,
- reference `s`, heading, curvature, lateral error, and mu,
- pitch/roll/ride-height readbacks from chassis.

### 11. Update Chassis Attitude

After tire forces determine `ax`, `ay`, and `yawAccel`, `SimpleChassis.updateFromAccelerations` advances platform attitude for the next step.

Heave is driven by aero downforce and resisted by platform stiffness/damping:

```text
heaveForce = Fz_aero_front + Fz_aero_rear
           - K_heave * heave
           - C_heave * heaveRate
```

Pitch moment includes inertial load transfer and aero pitch moment:

```text
M_pitch = sprungMass * ax * cg_height
        + (Fz_rear*a_rear - Fz_front*a_front)
        + F_drag * dragHeight
        - K_pitch*pitch
        - C_pitch*pitchRate
```

Roll is split into front and rear roll degrees of freedom. Each axle receives the lateral roll moment from its own axle-center lateral acceleration, is resisted by axle roll stiffness, and is coupled to the other axle by chassis torsional rigidity:

```text
ay_front = ay + yawAccel*frontArm
ay_rear = ay - yawAccel*rearArm
twist = frontRoll - rearRoll
M_front_roll = sprungMass*frontWeight*ay_front*cg_height
             - K_roll_front*frontRoll
             - C_roll_front*frontRollRate
             - K_torsion*twist
             - C_torsion*twistRate
M_rear_roll = sprungMass*rearWeight*ay_rear*cg_height
             - K_roll_rear*rearRoll
             - C_roll_rear*rearRollRate
             + K_torsion*twist
             + C_torsion*twistRate
```

When `yawAccel = 0`, both axles see the CG lateral acceleration and this collapses to the previous scalar-`ay` behavior. The legacy `rollAngle` telemetry is the average of front and rear roll.

### 12. Project To Reference Track

In track mode, the new free-space `x, y` is projected to the closest reference segment. The projection returns:

- progress `s`,
- reference heading,
- interpolated curvature,
- interpolated surface mu,
- signed lateral error,
- track-limit margin and `onTrack`.

The projection first searches near the previous segment for speed. If the nearest local result is suspicious, it falls back to a full-track search. Projection updates reference telemetry only; it does not move the vehicle back onto the track.

In free mode, used by correlation replay, progress is the world distance travelled and there is no track-limit check.

### 13. Log Telemetry

`simulate` logs a wide state struct. Full telemetry includes:

- body state: position, velocity, yaw, CG and axle lateral accelerations, body slip,
- controls and driver targets,
- reference projection and track margin,
- aero forces and pitch moments,
- drivetrain torques and RPM,
- per-corner suspension states,
- per-corner tire loads, slip, omega, `Fx`, `Fy`,
- chassis pitch, roll, twist, and ride height.

Lean telemetry keeps only the hot-path channels used for profiling.

## Correlation Replay Flow

Correlation replay is intentionally a free-space input replay by default.

1. `scripts/extract_motec_lap.py` or a provided CSV normalizes logged channels.
2. `CorrelationReplayProfile` validates and interpolates time, distance, throttle, brake, pressure, steer, speed, and optional yaw/position/acceleration channels.
3. `CorrelationStateInitializer` seeds `VehicleState` from logged speed, velocity/body slip, yaw, yaw rate, and position when present.
4. `Simulator.simulateReplay` temporarily installs `TelemetryReplayDriver`, replay brake mode, replay stop conditions, and free/track reference mode.
5. The same `Simulator.step` handles aero, suspension, tires, powertrain, differential, brakes, and state integration.

This means correlation overlays answer: "What path and acceleration does this vehicle model produce when driven with the logged controls?" They do not force the simulated car to match logged GPS.

## Important Assumptions

- The vehicle is a planar rigid body for x/y/yaw, with a separate lumped sprung-mass attitude model for heave/pitch/roll.
- The drivetrain is rear-wheel drive.
- The default aero model is one whole-car resultant, although component aero classes exist.
- The tire model is Pacejka/MFeval and is the source of grip and load sensitivity.
- Surface mu scales the tire output relative to the reference tire file; it is not an independent hard friction cap.
- Brake commands generate wheel torque; tire slip decides the resulting ground force.
- The track centerline is not a rail.
- Road height is flat; vertical dynamics are suspension/tire compliance about a flat road.
- Thermal tire behavior, aero yaw sensitivity, ABS, detailed inverter dynamics, and 3D road surface are not modeled.

## Reading The Code By Physics Topic

Start with `Simulator.step` for the full force path. Then read:

- Aero equations: `src/+components/+Aero/WholeCarAero.m` and `AeroComponent.m`.
- Load transfer and suspension kinematics: `SuspensionManager.m`, `SimpleSuspension.m`, `SuspensionGeometry.m`.
- Chassis attitude: `SimpleChassis.m` and `ChassisState.m`.
- Wheel slip and tire forces: `PacejkaTire.m` and `TireState.m`.
- Motor torque and differentials: `EMRAX228Powertrain.m`, `DifferentialComponent.m`, and the concrete differential files.
- Driver targets: `DriverInputPlanner.m` and `DriverModel.m`.
- Track reference and projection: `Track.m`, `WaypointTrack.m`, and `Simulator.projectToReference`.
- Replay: `run_correlation.m`, `CorrelationReplayProfile.m`, and `TelemetryReplayDriver.m`.
