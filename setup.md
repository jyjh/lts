# Setting up the initial car model

This guide covers the work needed before calibration.

The aim is to create the best model possible from:

- direct measurements;
- CAD and design calculations;
- supplier data;
- tire, damper, motor and aero test data;
- clearly recorded engineering estimates.

Calibration should improve uncertain physical values. It should not repair
incorrect units, missing components or values copied from the wrong car.

The examples use an imaginary `R26` car.

## 1. Collect the source information

Create one folder or engineering record containing the information used for
the car model.

Recommended inputs are:

| Information | Typical source |
|---|---|
| FSAE design spec sheet | Submitted design document |
| Vehicle mass and corner weights | Setup scales |
| Component masses and locations | CAD or measured parts |
| Suspension geometry | CAD hardpoints or kinematic software |
| Springs and ARBs | Part specifications and bench measurements |
| Dampers | Damper dyno |
| Tires | Correct `.tir` file and tire test information |
| Aero forces and balance | CFD, wind tunnel or coastdown testing |
| Motor and inverter | Supplier maps and logged shaft torque |
| Gear ratio and differential | Drawings and measured hardware |
| Brake system | Hydraulic calculations and pressure testing |
| Sensor calibration | Logger configuration and calibration tests |

For every value, record:

- the value;
- units;
- source;
- date or revision;
- estimated uncertainty;
- whether it is measured, calculated or assumed.

Do not keep the source only in a private spreadsheet. The value and its source
should also be understandable from the vehicle file or parameter manifest.

## 2. Generate the vehicle file

Export the FSAE design spec sheet as CSV.

Preview the result:

```powershell
python scripts/generate_vehicle.py `
  "data/2026_FSAE_Design_EV_Spec_Sheet.csv" `
  --name R26 `
  --driver-mass 68 `
  --dry-run
```

The preview reports:

- values mapped directly into the model;
- values that could be calculated but are ambiguous;
- spec-sheet information that has no matching model field;
- model values missing from the spec sheet.

Create the file after checking the preview:

```powershell
python scripts/generate_vehicle.py `
  "data/2026_FSAE_Design_EV_Spec_Sheet.csv" `
  --name R26 `
  --driver-mass 68
```

The output is:

```text
src/+lts/+vehicles/R26.m
```

The generated file starts with:

```matlab
function cfg = R26()
    cfg = lts.vehicle.VehicleConfig();
    cfg.name = "R26";
```

It then replaces the default values with information found in the spec sheet.

The generator does not remove the need for an engineering review. The spec
sheet covers only part of the model.

## 3. Understand the coordinate system and units

The vehicle model uses SI units.

| Quantity | Unit |
|---|---|
| Length | m |
| Mass | kg |
| Force | N |
| Torque | N·m |
| Angle | rad |
| Spring rate | N/m |
| Damping | N·s/m |
| Pressure calibration | N/bar |
| Rotational inertia | kg·m² |

Vehicle axes are:

- `x`: forward;
- `y`: left;
- `z`: upward.

Positions such as aero center of pressure are measured relative to the vehicle
CG unless the field says otherwise.

Common conversion mistakes include:

- entering mm as m;
- entering N/mm as N/m;
- entering degrees where radians are required;
- using vehicle mass without the driver;
- using tire diameter as tire radius;
- using motor torque where wheel torque is expected.

Check every copied value against the unit shown in `VehicleConfig.m`.

## 4. Vehicle mass properties

The main fields are:

```matlab
cfg.totalMass
cfg.wheelbase
cfg.trackWidth
cfg.cgHeight
cfg.staticFrontWeight
cfg.yawInertia
cfg.unsprungMass
```

### 4.1 Total mass

`totalMass` includes the driver and the car in the condition being simulated.

Record:

- vehicle mass without driver;
- driver mass;
- fluids;
- accumulator state;
- ballast;
- any equipment present during the logged test.

Do not compare a simulation containing a 68 kg driver with a test run using a
different driver without updating the mass and location.

### 4.2 Wheelbase and track width

Use installed dimensions, not only nominal CAD values.

The current model has one `trackWidth` value. If front and rear tracks differ,
record that limitation. Use the dimension that best represents the model until
separate front and rear track widths are supported.

### 4.3 Static weight distribution

`staticFrontWeight` is the fraction of total weight on the front axle:

```text
front fraction = front axle weight / total vehicle weight
```

For example, 50.8% front weight is entered as:

```matlab
cfg.staticFrontWeight = 0.508;
```

Measure it with the driver installed and the car in its test condition.

### 4.4 CG height

`cgHeight` is measured upward from the ground.

Preferred methods are:

- a tilt-table or raised-axle test;
- a validated CAD mass model;
- a combination of component mass measurements and CAD positions.

Record the tire radius and suspension condition used during a physical CG
test. They affect the result.

### 4.5 Yaw inertia

`yawInertia` is the rotational inertia about the vertical axis through the CG.

Preferred sources are:

- bifilar or torsional pendulum test;
- validated CAD mass properties;
- component-by-component mass model.

A simple estimate based only on total mass and wheelbase is suitable as a
starting uncertainty range, not as a high-confidence measurement.

Yaw inertia strongly affects transient yaw response. Do not use it as a free
knob to hide incorrect tire stiffness or steering calibration.

### 4.6 Unsprung mass

`unsprungMass` is per corner.

Include the appropriate share of:

- wheel and tire;
- upright;
- hub and bearing;
- brake rotor and caliper;
- outboard suspension parts;
- part of the driveshaft or axle where applicable.

Front and rear unsprung masses may differ. The current configuration has one
value, so document any averaging.

## 5. Aerodynamics

The main fields are:

```matlab
cfg.aero.ClA
cfg.aero.CdA
cfg.aero.xPosition
cfg.aero.zPosition
cfg.aero.pitchSensitivityClA
```

### 5.1 Use coefficient-area values

The model uses `ClA` and `CdA`, not coefficient alone.

At speed `V`:

```text
downforce = 0.5 × air density × ClA × V²
drag      = 0.5 × air density × CdA × V²
```

If CFD provides force at a known speed:

```text
ClA = 2 × downforce / (air density × V²)
CdA = 2 × drag      / (air density × V²)
```

Check:

- whether reported lift uses a positive or negative sign;
- reference area;
- air density;
- ride height;
- yaw angle;
- wheel rotation and moving-ground settings.

The model expects positive `ClA` to produce downforce.

### 5.2 Aero balance and longitudinal position

`xPosition` is the longitudinal center of pressure relative to the vehicle CG:

- positive is forward;
- negative is rearward.

For a known front aero fraction:

```text
xPosition = wheelbase × (front aero fraction - static front weight fraction)
```

This convention matches the model's load distribution.

Check the result against the physical layout. A center of pressure far outside
the wheelbase usually indicates a percentage or sign error.

### 5.3 Aero force height

`zPosition` is the height of the aero resultant above the ground.

Setting it equal to `cgHeight` removes the pitch moment caused by drag. This is
an assumption, not a measurement.

Use a physical force height when it is available and when the pitch model can
represent it correctly.

### 5.4 Pitch or ride-height sensitivity

`pitchSensitivityClA` changes downforce as the car pitches.

Leave it at zero unless there is an aero sweep or other evidence. A single CFD
point cannot define pitch sensitivity.

For an aero design study, change the full aero description:

- `ClA`;
- `CdA`;
- aero balance;
- force height;
- pitch or ride-height behaviour.

Do not change downforce alone.

## 6. Suspension springs, dampers and anti-roll bars

The main spring and damper fields are:

```matlab
cfg.suspension.front.springRate
cfg.suspension.front.dampingCoeff
cfg.suspension.front.reboundCoeff

cfg.suspension.rear.springRate
cfg.suspension.rear.dampingCoeff
cfg.suspension.rear.reboundCoeff

cfg.suspension.motionRatio
cfg.suspension.bumpStopLength
cfg.suspension.bumpStopRate
cfg.suspension.tireSpringRate
```

### 6.1 Spring rate and wheel rate

Confirm whether the source gives:

- spring rate;
- wheel rate;
- axle roll rate.

They are not interchangeable.

The model uses:

```text
wheel rate = spring rate × motion ratio²
```

Check the motion-ratio definition used by the suspension team. Some software
uses wheel movement divided by spring movement, while other software uses the
inverse.

Use one known suspension movement to confirm the conversion.

### 6.2 Dampers

Use damper dyno data where possible.

The current simple model uses one bump coefficient and one rebound coefficient
per axle. Real dampers are nonlinear and may have low-speed and high-speed
regions.

Choose the part of the dyno curve that matches the wheel velocities expected
in the test, then record:

- damper setting;
- gas pressure;
- test temperature;
- whether the value is at the damper or wheel;
- motion-ratio conversion.

Do not convert a “percentage of critical damping” into a coefficient without
checking the mass and spring rate used in that calculation.

### 6.3 Bump stops and tire vertical stiffness

Record:

- free travel before bump-stop contact;
- bump-stop force curve;
- installed preload;
- tire vertical stiffness at the operating pressure.

If only one bump-stop rate is available, state the travel range over which the
linear approximation is intended to work.

### 6.4 Anti-roll bars

Each anti-roll bar has:

```matlab
stiffness
motionRatio
leverArm
enabled
```

Check whether the supplied stiffness is:

- torsional bar stiffness;
- drop-link rate;
- wheel rate;
- full axle roll stiffness.

The model expects torsional stiffness together with lever arm and motion ratio.

Set `enabled = false` when no bar is fitted. Do not leave a default bar enabled
on a car that has no ARB.

## 7. Suspension and steering geometry

Geometry is stored separately for the front and rear axles.

Important fields include:

```matlab
travelGrid
camberCurve
toeCurve
motionRatioCurve
rollCenterHeight
rollCenterLateral
casterAngle
mechanicalTrail
scrubRadius
kingpinInclination
kingpinOffset
```

The curves are values against wheel travel.

Before entering them:

1. confirm the zero-travel position;
2. confirm whether bump is positive or negative;
3. confirm camber and toe sign;
4. convert degrees to radians;
5. check left and right wheel conventions;
6. cover the full expected travel range.

Plot the curves after entering them. Sudden sign changes or values ten times
larger than expected are often unit errors.

Steering fields include:

```matlab
cfg.suspension.geometry.steering.steeringRatio
cfg.suspension.geometry.steering.ackermann
cfg.suspension.geometry.steering.maxWheelSteerAngle
cfg.suspension.geometry.steering.rearSteerRatio
```

The simulation steering input is road-wheel angle unless the logger conversion
states otherwise.

Do not put steering-wheel angle directly into a road-wheel-angle field.

Validate steering using:

- steering-wheel angle versus left and right road-wheel angle;
- several points across the rack travel;
- a center check for sensor offset or nonlinearity.

## 8. Chassis stiffness and motion

The chassis block contains:

```matlab
cfg.chassis.heaveStiffness
cfg.chassis.heaveDamping
cfg.chassis.pitchStiffness
cfg.chassis.pitchDamping
cfg.chassis.rollStiffness
cfg.chassis.rollDamping
cfg.chassis.torsionalRigidity
cfg.chassis.torsionalDamping
```

Use physical test data where possible.

Be careful with torsional rigidity units:

```text
N·m/degree × 180/π = N·m/rad
```

Avoid counting the same stiffness twice. For example, if suspension springs and
ARBs already produce the roll stiffness, do not also force a duplicate chassis
roll stiffness into load transfer without understanding how the model combines
them.

## 9. Powertrain and differential

Important fields include:

```matlab
cfg.powertrain.matFile
cfg.powertrain.finalDriveRatio
cfg.powertrain.efficiency
cfg.powertrain.motorRotorInertia
cfg.powertrain.throttleMapInput
cfg.powertrain.throttleMapOutput
cfg.powertrain.differential
```

Some fields are optional and use component defaults when absent.

### 9.1 Motor map

Check:

- torque units;
- motor speed units;
- continuous versus peak limits;
- voltage used for the map;
- inverter current limits;
- thermal limits;
- positive and negative torque signs.

Use shaft torque for the motor map. Apply the final drive and drivetrain
efficiency separately.

### 9.2 Final drive

Confirm the full motor-to-wheel ratio, including every gear stage.

Check it against logged motor RPM and measured vehicle or driven-wheel speed:

```text
wheel speed = motor speed / total gear ratio
```

Use the installed rolling radius for the speed comparison.

### 9.3 Efficiency and losses

Separate:

- inverter electrical efficiency;
- motor efficiency;
- gearbox or chain loss;
- differential and bearing loss;
- tire rolling resistance;
- aero drag.

Avoid one fitted efficiency value that changes sign incorrectly during
regeneration.

### 9.4 Differential

Set the actual differential type:

- open;
- locked;
- clutch LSD;
- Drexler ramp-plate LSD.

For an LSD, collect:

- preload or breakaway torque;
- acceleration ramp angle;
- deceleration ramp angle;
- plate arrangement;
- fluid and temperature;
- any measured torque-bias behaviour.

Do not copy LSD settings from another car because the same differential model
name is used.

## 10. Tires and rotating parts

Important fields are:

```matlab
cfg.tire.tirFile
cfg.tire.wheelRadius
cfg.tire.wheelInertia
cfg.tire.relaxationLength
cfg.tire.longitudinalRelaxationLength
cfg.tire.rollingResistanceCoeff
cfg.tire.bearingDragCoeff
```

### 10.1 Tire file

Use a `.tir` file for the correct:

- tire size;
- construction;
- compound;
- pressure range;
- normal-load range;
- test method.

Using a measured file from a different tire may be useful as an early estimate,
but it must be clearly recorded as a model limitation.

Do not quietly scale a tire file until it matches one lap. A fitted tire may
give the wrong answer when mass or downforce changes because tire load
sensitivity is central to both studies.

### 10.2 Rolling radius

`wheelRadius` is the effective installed rolling radius used by the vehicle
model.

Measure or calculate it under representative load and pressure.

The unloaded radius inside a tire file and the effective rolling radius are not
necessarily the same.

Check rolling radius using:

- marked-wheel distance measurement;
- wheel speed versus GPS speed;
- motor RPM, gear ratio and driven-wheel speed during low-slip running.

### 10.3 Wheel inertia

`wheelInertia` includes the rotating parts represented at each corner:

- tire;
- wheel;
- brake rotor;
- hub;
- driven axle contribution where appropriate.

Preferred sources are:

- pendulum or spin test;
- CAD inertia;
- component mass and radius calculation.

Do not use total wheel assembly mass as rotational inertia.

### 10.4 Relaxation lengths

Relaxation length controls how quickly tire force builds after slip changes.

Use tire test data or a justified geometric estimate. Keep lateral and
longitudinal values separate when the evidence supports it.

These values affect transient response and may be difficult to separate from
steering delay, yaw inertia and damper behaviour.

## 11. Brake model

The simple brake model uses:

```matlab
cfg.brakeBiasFront
cfg.brakeForceCoefficient
```

Pressure-based replay also uses:

```matlab
cfg.brakePressure.frontForcePerBar
cfg.brakePressure.rearForcePerBar
```

The force-per-bar values are total force for each axle.

They can be estimated from:

- line pressure;
- piston area;
- pad friction;
- effective disc radius;
- wheel radius;
- number of calipers and pistons.

Then check the estimate using a controlled braking test.

Do not normalize every lap's maximum pressure to 100% brake. That makes brake
response depend on which lap was selected.

## 12. Keep vehicle values separate from test conditions

The vehicle file should describe the car.

Test-day values belong in the scenario or dataset record:

- air density;
- weather;
- track condition;
- tire pressure and temperature;
- sensor offsets;
- logger timing;
- driver;
- test-specific ballast.

For example:

```matlab
scenario = struct('air_density', 1.18);
```

Do not change a permanent vehicle parameter simply because one test day had
different conditions.

## 13. Create the parameter manifest

After reviewing the vehicle file, create:

```text
config/governance/r26_parameter_manifest.json
```

For each important value, decide:

1. Is it a design variable?
2. Is it directly measured?
3. Is it an unknown physical value that can be calibrated?
4. Is it only a test-day condition?
5. Is it an old fitted value that must be kept out of design studies?

Example:

```json
{
  "name": "rolling_resistance",
  "path": "tire.rollingResistanceCoeff",
  "role": "global_calibrated",
  "units": "ratio",
  "source": "Engineering estimate before coastdown testing",
  "provenance": "Initial estimate only",
  "uncertainty": {
    "distribution": "normal",
    "standardDeviation": 0.004
  },
  "calibrationDomain": {
    "lower": 0.003,
    "upper": 0.04
  }
}
```

The uncertainty should describe what is genuinely known before using the test
that will calibrate it.

Do not give every parameter a very wide range. Wide ranges allow the fitting
process to use one subsystem to hide an error in another.

## 14. Run checks before calibration

### 14.1 Build the governed vehicle

```matlab
addpath('src')

governed = lts.governance.GovernedVehicle.build( ...
    @lts.vehicles.R26, ...
    'config/governance/r26_parameter_manifest.json', ...
    'config/governance/r26_provisional_calibration.json', ...
    'Scenario', struct('air_density', 1.18));
```

This checks:

- manifest format;
- parameter roles;
- calibration-file compatibility;
- parameter ranges;
- prohibited legacy calibration.

### 14.2 Run simple tracks

Use simple tests before a full lap:

```matlab
straight = lts.components.TestTrack('straight75');
skidpad = lts.components.TestTrack('skidpad');
```

Check that:

- the car accelerates in the correct direction;
- motor RPM matches vehicle speed and gear ratio;
- wheel loads are reasonable;
- downforce and drag have the expected signs;
- braking slows the car;
- steering produces yaw in the expected direction;
- tire forces and slip are finite;
- the car sits at a sensible static ride height.

### 14.3 Check orders of magnitude

Make a short engineering calculation outside the simulator:

- static axle loads;
- expected downforce and drag at 15, 20 and 25 m/s;
- wheel force from motor torque;
- motor RPM at several vehicle speeds;
- brake force at a known line pressure;
- steady lateral acceleration from the tire data;
- spring deflection under static load.

Compare these with simulator telemetry. This catches many setup errors faster
than a full-lap comparison.

### 14.4 Review plots, not only laptime

Review:

- speed;
- motor RPM and torque;
- wheel speeds and slip;
- tire normal loads;
- tire forces and utilization;
- aero forces;
- brake force;
- pitch and roll;
- suspension travel;
- steering and yaw response.

A plausible laptime can still come from incorrect forces that happen to cancel.

## 15. Decide whether the initial model is ready

The car is ready for calibration when:

- all important units and signs have been checked;
- the car configuration matches the test car;
- measured values are not being fitted;
- uncertain values have realistic ranges;
- test-day conditions are kept outside the permanent car definition;
- aero and mechanical losses are separate;
- tire data and limitations are clearly recorded;
- simple straight-line, braking and cornering behaviour is physically sensible;
- every remaining calibration parameter has a planned test that excites it.

If a parameter has no suitable measurement or test, leave it uncertain. Do not
claim that a full-lap fit has measured it.

Continue with [workflow.md](workflow.md) for test-data preparation,
calibration, validation and design-change prediction.

## Initial model checklist

- [ ] Spec-sheet CSV generated and reviewed.
- [ ] Every `TODO` and default value reviewed.
- [ ] SI units and sign conventions checked.
- [ ] Mass includes the correct driver and test condition.
- [ ] Corner weights and CG information recorded.
- [ ] Yaw inertia source and uncertainty recorded.
- [ ] Spring rate, wheel rate and motion ratio are not confused.
- [ ] Damper coefficients match the installed setting and velocity range.
- [ ] Suspension geometry curves plotted and checked.
- [ ] Steering-wheel and road-wheel angles are not confused.
- [ ] Aero uses `ClA` and `CdA`, with correct balance and force height.
- [ ] Motor map, gear ratio and wheel-speed relationship checked.
- [ ] Differential type and settings match the installed unit.
- [ ] Correct tire data used, or substitute tire limitations clearly stated.
- [ ] Rolling radius and wheel inertia checked independently.
- [ ] Brake force and pressure calibration checked.
- [ ] Vehicle properties separated from test-day conditions.
- [ ] Parameter sources, uncertainty and roles recorded.
- [ ] Governed vehicle builds without legacy tuning.
- [ ] Simple-track telemetry is physically sensible.
- [ ] Initial model limitations are written down.
