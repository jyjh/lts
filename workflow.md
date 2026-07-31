# Example workflow: from car data to a design decision

This guide shows how to:

1. build a model of the current car;
2. improve the model using test data;
3. check the model against data that was not used for calibration;
4. predict the laptime effect of a design change.

The most important rule is:

> Calibrate the current car once. Make the proposed change in the model, then
> run it again. Do not recalibrate the changed car.

If the changed car needs a different calibration to give a believable result,
the model is not yet ready to predict that change.

The examples below use an imaginary `R26` car. Replace the names and values
with those for your car.

For a detailed subsystem-by-subsystem guide to the initial model, including
units, sign conventions and common checks, see [setup.md](setup.md).

## 1. Build the first car model

### 1.1 Generate a model from the design spec sheet

Export the FSAE design spec sheet as a CSV file.

First, preview the generated model without creating a file:

```powershell
python scripts/generate_vehicle.py `
  "data/2026_FSAE_Design_EV_Spec_Sheet.csv" `
  --name R26 `
  --driver-mass 68 `
  --dry-run
```

If the preview looks reasonable, create the model:

```powershell
python scripts/generate_vehicle.py `
  "data/2026_FSAE_Design_EV_Spec_Sheet.csv" `
  --name R26 `
  --driver-mass 68
```

This creates:

```text
src/+lts/+vehicles/R26.m
```

The generated file is only a starting point. The spec sheet does not contain
everything needed by the simulation.

Search the generated file for:

- `TODO`
- `[not in spec sheet]`
- comments describing an uncertain conversion or assumption

These values need to be checked before relying on the model.

### 1.2 Add measurements from the real car

Use measurements instead of fitted values wherever possible.

| Area | Useful information |
|---|---|
| Mass | Total mass with driver, four-corner weights, component masses and locations |
| CG and inertia | CG height test, weight distribution, yaw inertia test or CAD estimate |
| Suspension | Wheel rates, motion ratios, damper dyno data, ARB rates, kinematic curves |
| Tires | Correct tire data, test pressure, temperature range and tire age |
| Aerodynamics | `ClA`, `CdA`, aero balance and ride-height or pitch sensitivity |
| Powertrain | Motor map, gear ratio, shaft torque and drivetrain efficiency |
| Brakes | Line pressure, brake bias and measured force per bar |
| Mechanical losses | Coastdown data, rolling resistance, bearing drag and drivetrain loss |

Keep different losses separate. For example, do not increase `CdA` to make up
for missing bearing or drivetrain losses. That may match the current car, but
it will give the wrong result when the aero package changes.

Check that MATLAB can build the car:

```matlab
addpath('src')

cfg = lts.vehicles.R26();
track = lts.components.TestTrack('straight10');
vehicle = lts.vehicle.VehicleManager.fromConfig(cfg, track, 0.001);
```

If this fails, correct the vehicle data before moving on.

## 2. Record where every important value came from

The project uses a parameter manifest. This is a JSON file that records:

- the parameter name and units;
- where the value came from;
- how uncertain it is;
- whether calibration is allowed to change it;
- the range of conditions over which it is believed to be valid.

The code uses the word `provenance` for the record of where a value came from.

Start by copying:

```text
config/governance/r25_parameter_manifest.json
```

Create:

```text
config/governance/r26_parameter_manifest.json
```

Update the vehicle name, ID, sources, uncertainty and valid ranges.

Each parameter has one of these roles:

| Role | Meaning |
|---|---|
| `design` | A value that may be changed in a design study, such as mass or aero |
| `fixed_measured` | A measured value that calibration is not allowed to change |
| `global_calibrated` | An unknown physical value that may be found from test data |
| `run_nuisance` | A test-day condition, such as air density |
| `legacy_effective` | An old fitted value kept only for comparison |

Only `global_calibrated` values may be fitted.

A parameter should be `global_calibrated` only if it describes the same
physical property before and after the proposed design change.

## 3. Create the starting calibration file

The calibration file stores the current best estimates for the unknown
physical parameters. The code calls this a calibration artifact.

Before using test data, it may contain engineering estimates:

```json
{
  "schema": "lts.governance.calibration-artifact.v1",
  "id": "r26-physical-priors-v1",
  "manifestId": "r26-governed-v1",
  "certification": "provisional",
  "parameters": [
    {"name": "yaw_inertia", "value": 135.0},
    {"name": "rolling_resistance", "value": 0.015}
  ],
  "sourceDatasetIds": [],
  "provenance": "Measurements and engineering estimates only",
  "legacy": false
}
```

Save this as:

```text
config/governance/r26_provisional_calibration.json
```

`provisional` means the file has not yet been proven against independent test
data.

Check that the car, parameter list and calibration file work together:

```matlab
governed = lts.governance.GovernedVehicle.build( ...
    @lts.vehicles.R26, ...
    'config/governance/r26_parameter_manifest.json', ...
    'config/governance/r26_provisional_calibration.json', ...
    'Scenario', struct('air_density', 1.18));
```

## 4. Plan useful vehicle tests

A full lap contains many effects at the same time. This makes it difficult to
tell whether an error comes from the tires, aero, powertrain or driver input.

Use simple tests to separate the main effects:

1. **Sensor checks**  
   Check units, signs, zeros, delays, steering angle, brake pressure and wheel
   speed calibration.

2. **Coastdown tests**  
   Use these to study aero drag, rolling resistance, bearing drag and other
   mechanical losses.

3. **Straight acceleration tests**  
   Use these to study delivered torque, drivetrain efficiency, rotating
   inertia and longitudinal tire behaviour.

4. **Controlled braking tests**  
   Use these to study brake force per bar and front-to-rear balance.

5. **Constant-radius, step-steer or slalom tests**  
   Use these to study lateral tire response, yaw inertia and transient
   response.

6. **Full laps**  
   Use these to check the complete vehicle model.

7. **A known A/B change**  
   Use this to check whether the model can predict a modified car. Adding or
   removing known ballast is a good first test.

For every run, record:

- the exact car setup;
- driver and ballast;
- tire type, pressure, temperature and age;
- weather and track condition;
- sensor and channel-map version;
- the purpose of the run;
- any difference from the baseline car.

Without this information, differences between runs may be incorrectly blamed
on the vehicle model.

## 5. Convert and catalogue the test data

Convert each run separately:

```matlab
addpath('src')

out = lts.app.preprocess_plant_data( ...
    'MoTeCFile', 'data/r26_coastdown_01.ld', ...
    'LdxFile', 'data/r26_coastdown_01.ldx', ...
    'OutputBase', 'exports/plant_data/r26_coastdown_01');
```

This creates:

- a replay CSV used by the simulator;
- an extraction report describing how the log was converted.

Plot or inspect the replay before calibration. Fix incorrect units, signs,
timing or channels at this stage. Do not use vehicle parameters to hide a data
problem.

Add the run to the dataset catalogue:

```text
config/governance/r26_dataset_catalog.json
```

Example entry:

```json
{
  "id": "r26-coastdown-01",
  "role": "calibration",
  "vehicleConfiguration": "R26 baseline",
  "testDay": "2026-08-15",
  "sourceFile": "data/r26_coastdown_01.ld",
  "sha256": "<file hash>",
  "conditions": {
    "airDensityKgM3": 1.18,
    "tirePressureKPa": 83,
    "surface": "dry"
  },
  "intervention": {}
}
```

The file hash confirms that the raw log has not changed.

On Windows, obtain it with:

```powershell
Get-FileHash "data/r26_coastdown_01.ld" -Algorithm SHA256
```

Choose one role for each complete run:

| Role | Use |
|---|---|
| `calibration` | May be used to find unknown parameters |
| `baseline-validation` | Unchanged-car run kept out of calibration |
| `transport-validation` | Known vehicle change kept out of calibration |

Do not use one part of a lap for calibration and a nearby part of the same lap
as proof that the model works. They share the same conditions and are not
independent tests.

## 6. Calibrate the unknown physical values

Calibrate one group of related values at a time:

1. sensor and timing checks;
2. longitudinal losses;
3. powertrain and braking;
4. lateral and yaw response.

The calibration code needs a residual function. This is a small MATLAB
function that:

1. runs the relevant test with a candidate vehicle model;
2. compares the simulation with the measurement;
3. returns the errors as a list of numbers.

For example:

```matlab
lossStage = struct( ...
    'name', "longitudinal_losses", ...
    'parameters', ["rolling_resistance", "bearing_drag"], ...
    'residual', @(cfg) coastdownResidual(cfg, coastdownExperiment));
```

`coastdownResidual` is a team-written function for that experiment. It could
return speed error or deceleration error at each measurement point.

Run the calibration:

```matlab
result = lts.app.calibrate_plant( ...
    'VehicleConfig', @lts.vehicles.R26, ...
    'Manifest', 'config/governance/r26_parameter_manifest.json', ...
    'DatasetCatalog', 'config/governance/r26_dataset_catalog.json', ...
    'DatasetIds', "r26-coastdown-01", ...
    'Stages', lossStage, ...
    'ArtifactId', 'r26-calibration-v1', ...
    'OutputFile', 'config/governance/r26_calibration_v1.json');
```

The calibration code checks:

- whether fitting the chosen parameter is allowed;
- whether the test is sensitive to that parameter;
- whether two parameters have nearly the same effect;
- whether the fitted value remains inside its allowed range.

If two parameters cannot be separated, do not simply accept the best numerical
fit. Measure one of them, run a better test, or keep the uncertainty in the
final prediction.

The new calibration file still starts as `provisional`.

## 7. Check the model against an unused run

Use a complete run that was not used for calibration:

```matlab
report = lts.app.run_plant_validation( ...
    'ReplayCsv', 'exports/plant_data/r26_validation_lap_replay.csv', ...
    'Manifest', 'config/governance/r26_parameter_manifest.json', ...
    'CalibrationArtifact', 'config/governance/r26_calibration_v1.json', ...
    'Track', '2026enduro', ...
    'Scenario', struct('air_density', 1.18));
```

Check:

- position and speed error;
- yaw-rate and acceleration error;
- wheel-speed error;
- whether the error changes with speed, throttle, brake or steering;
- approximate energy balance;
- warnings that the car is outside the tested range;
- `initializationCount == 1`;
- `stateResetCount == 0`.

The whole run is started once. The simulation is not repeatedly reset to the
measured car state.

Short reset-and-replay sections are still useful for finding local problems,
but they are not the main proof that the model works.

A successful unused baseline run can support `baseline-validated` status. It
does not yet prove that the model can predict a design change.

## 8. Describe the proposed design change

The change file should describe the real physical change, including connected
effects.

### Example: remove 5 kg

```json
{
  "schema": "lts.prediction.design-change.v1",
  "id": "remove-5kg-accumulator-support",
  "source": "Released CAD mass properties",
  "provenance": "Revision C compared with the installed baseline",
  "operations": [
    {
      "type": "mass",
      "deltaMassKg": -5.0,
      "xFromCgM": -0.20,
      "yFromCgM": 0.0,
      "zFromGroundM": 0.18
    }
  ]
}
```

The mass and location are both required. The code then updates:

- total mass;
- CG position;
- weight distribution;
- yaw inertia.

### Example: new aero package

```json
{
  "type": "aero_map",
  "ClA": 5.30,
  "CdA": 1.95,
  "xPosition": -0.04,
  "zPosition": 0.26,
  "pitchSensitivityClA": 0.0
}
```

Do not enter only “10% more downforce.” The added drag, aero balance and
ride-height or pitch effects are also needed.

## 9. Predict the laptime change

Run the baseline and changed car together:

```matlab
study = lts.app.predict_design_change( ...
    'VehicleConfig', @lts.vehicles.R26, ...
    'Manifest', 'config/governance/r26_parameter_manifest.json', ...
    'CalibrationArtifact', 'config/governance/r26_calibration_v1.json', ...
    'Change', 'config/design_changes/r26_remove_5kg.json', ...
    'Track', '2026enduro', ...
    'Scenario', struct('air_density', 1.18), ...
    'SampleCount', 10, ...
    'Seed', 26, ...
    'AllowProvisional', true);
```

Start with a small `SampleCount`. Increase it after checking that both
simulations complete correctly.

The same uncertain parameter values are used for each baseline/changed pair.
This makes the uncertainty in the laptime difference more useful.

Review:

```matlab
study.baseline.lapTime
study.variant.lapTime
study.deltaLapTime
study.sectorDelta
study.uncertainty.delta
study.sensitivityAttribution
study.domainWarnings
study.variantRefitted
study.decisionStatus
```

Important checks:

- both baseline and changed runs must be feasible, meaning they finish without
  leaving the track or failing numerically;
- `variantRefitted` must be `false`;
- the result must not be outside the tested range without a warning;
- provisional studies remain `uncertified`.

`AllowProvisional=true` allows an early engineering study. It does not mean
that the answer has been proven.

## 10. Test the prediction on the real car

The first recommended test is a known ballast change:

1. Measure the ballast mass and position.
2. Keep tires, setup, driver and conditions as similar as possible.
3. Alternate baseline and ballast runs to reduce the effect of changing track
   conditions.
4. Make the simulation prediction before checking the measured laptime change.
5. Do not recalibrate the ballast version of the car.
6. Mark the A/B runs as `transport-validation` in the dataset catalogue.

The current pass targets are:

- absolute laptime error no greater than 2%;
- correct direction of the laptime change;
- change error no greater than the larger of:
  - 20% of the measured change;
  - 0.2 seconds;
- no recalibration of the changed car.

Example:

```matlab
baselineEvidence = struct('absoluteErrorFraction', 0.015);

transportEvidence = struct( ...
    'measuredDeltaS', 0.62, ...
    'predictedDeltaS', 0.55, ...
    'variantRefitted', false);

gate = lts.validation.CertificationGate.assess( ...
    'config/governance/r26_dataset_catalog.json', ...
    baselineEvidence, ...
    transportEvidence);
```

A passing A/B test can support `transport-validated` status.

After ballast, test another part of the model. Aero-trim and power-limit tests
are useful because they check different physical effects.

## 11. If the prediction is wrong

Do not immediately fit the changed car.

Check:

1. Were the test conditions actually the same?
2. Are the sensors, signs, units and timing correct?
3. Does the change file describe the complete physical change?
4. Is the changed car outside the range covered by the tests?
5. Is an important physical effect missing?
6. Were two fitted parameters impossible to separate?
7. Did both laptime simulations complete without leaving the track?

Add better physical data to the baseline model, then rerun both cars with the
same updated calibration.

## Final checklist

- [ ] Generated car model reviewed.
- [ ] Unknown defaults and assumptions identified.
- [ ] Important values have a source and uncertainty.
- [ ] Aero, tire and mechanical losses kept separate.
- [ ] Simple tests used to isolate different parts of the car.
- [ ] Calibration and validation use different complete runs.
- [ ] Raw log file hashes recorded.
- [ ] Only allowed physical parameters fitted.
- [ ] Parameters can be separated by the chosen tests.
- [ ] Unused baseline run checked without repeated state resets.
- [ ] Design change includes connected effects such as CG, inertia and drag.
- [ ] Baseline and changed car use the same calibration.
- [ ] Both laptime simulations are feasible.
- [ ] Uncertainty and range warnings reviewed.
- [ ] Result remains uncertified until a no-refit A/B test passes.
