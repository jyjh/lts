# scripts/

Code-generation and analysis tooling for the FSAE lap-time simulator.

## MATLAB utility scripts

Standalone diagnostic scripts. Run them by name from MATLAB — each auto-addpaths
`src/` and writes a PNG to `exports/` (created if missing). Set `configName` at
the top to switch vehicle configs.

- **`powertrain_power_curve.m`** — plots the configured powertrain's motor
  torque, motor power, wheel tractive force, and wheel torque curves, sampled via
  the `EMRAX228Powertrain` public API over a dense RPM grid (so the
  constant-power rolloff and rev-limit cut match what the simulator sees). Prints
  peak torque/power/force and the map-end / rev-limit speeds to the console.
  Saves `exports/powertrain_power_curve.png`.
- **`theoretical_acceleration_75m.m`** — best-case 75 m straight-line
  acceleration estimate, full-throttle force capped by rear-axle tire grip from
  the active `.tir` file. Saves
  `exports/theoretical_acceleration_75m_diagnostics.png`.
- **`tire_sensitivity.m`** — loads a Pacejka `.tir` file and creates stacked
  lateral- and longitudinal-force sensitivity plots over normal load for
  selected slip angles and positive drive slip ratios. Also plots best
  steady-state skidpad acceleration over vehicle mass with a simple bicycle
  load-transfer model and marks a diminishing-returns point from the
  acceleration-vs-mass curve (true inflection when present, knee fallback
  otherwise). The mass curve is sampled independently at `MassStepKg = 0.1`
  kg and brute-force searches `MassSlipAnglesDeg = 0:0.1:14` by default. Saves
  `exports/tire_lateral_sensitivity.png`,
  `exports/tire_longitudinal_sensitivity.png`, and
  `exports/tire_acceleration_vs_mass.png` by default.
- **`weight_savings_skidpad.m`** — sweeps vehicle mass and finds the point of
  diminishing returns for weight savings in terms of FSAE skidpad lap time,
  using a simplified steady-state bicycle load-transfer model (aero neglected)
  fed by the active `.tir` file's peak lateral-force envelope. At each mass the
  max `a_y` is solved by bisection on the load-transfer capacity residual;
  lap time uses the FSAE figure-8 constant `T = 5.9527 / sqrt(a_y)`. The
  diminishing-returns mass is the Kneedle elbow on the lap-time-vs-mass curve
  (raw G itself has *accelerating* returns to lightness because tire μ rises
  as load falls; the `1/sqrt(G)` compression in lap time is what produces
  genuine diminishing returns). Plots three panels (sustained G, lap time,
  marginal lap-time benefit per kg saved). Defaults to the theoretical tire
  and R25 geometry (`CgHeight` 0.256 m, `StaticFrontWeight` 0.5095,
  `ReferenceMassKg` 264). Saves `exports/weight_savings_skidpad.png`.
- **`investigate_lateral_g.m`** — correlation sanity report for lateral
  acceleration. Compares raw MoTeC lateral accel, speed*yaw-rate, steering
  demand, simulated body/tire Ay, and tire-capacity/utilization. Pass
  `SimCsv`, `ReplayCsv`, and optionally `ReportFile`:

```matlab
investigate_lateral_g( ...
    'SimCsv', 'exports/correlation_run.csv', ...
    'ReplayCsv', 'exports/correlation_run_replay.csv', ...
    'ReportFile', 'exports/lateral_g_report.md')
```

## generate_vehicle.py

Parses an **FSAE (EV) Design Spec Sheet CSV** (the format is stable year-to-year)
and emits a MATLAB vehicle file under `src/+lts/+vehicles/<Name>.m`, structured
exactly like the reference configs (`lts.vehicles.baseline` / `lts.vehicle.VehicleConfig`).

The generated car is immediately usable: instantiate `lts.vehicle.VehicleConfig()`, then
override every value the spec sheet can speak to. Each field carries a trailing
comment recording where its value came from:

| Comment marker                        | Meaning                                                        |
|---------------------------------------|----------------------------------------------------------------|
| `[CSV rN: ...]`                       | Direct mapping — real value taken from the spec sheet          |
| `TODO derivable (...)`                | Derivable from the CSV but ambiguous; left at the baseline default |
| `[not in spec sheet]`                 | No CSV source exists; the baseline default is kept             |

### Usage

```bash
python scripts/generate_vehicle.py <spec_sheet.csv> [--name R26]
        [--driver-mass 68] [--output PATH] [--force] [--dry-run]
```

| Flag            | Default                          | Description                                                        |
|-----------------|----------------------------------|--------------------------------------------------------------------|
| `csv` (pos.)    | —                                | Path to the FSAE Design Spec Sheet CSV.                           |
| `--name`        | `generated`                      | Vehicle / function name -> `src/+lts/+vehicles/<Name>.m`. Must be a valid MATLAB identifier. |
| `--driver-mass` | `68`                             | Driver mass [kg] added to the car's no-driver mass for `totalMass`. |
| `--output`      | `src/+lts/+vehicles/<Name>.m`    | Override the output path.                                          |
| `--force`       | off                              | Overwrite an existing output file (refuses otherwise).            |
| `--dry-run`     | off                              | Print the generated file and the report; write nothing.           |

Standard library only — no dependencies.

### Example

```bash
python scripts/generate_vehicle.py "2026_FSAE_Design_EV_Spec_Sheet.csv" --name R26
```

writes `src/+lts/+vehicles/R26.m`, then reference it from `src/+lts/+app/run_simulation.m`:

```matlab
config = lts.vehicles.R26();
```

### What gets mapped

The tool always prints a sourcing report to stdout with four sections:

- **DIRECT MAPPINGS APPLIED** — values written into the config (with unit
  conversions, e.g. mm→m, N/mm→N/m, N·m/deg→N·m/rad, %→fraction).
- **DERIVED (left as TODO)** — present in the CSV but ambiguous; left at the
  baseline default with the derivation sketched in a comment.
- **UNMAPPED** — present in the spec sheet but with no field in `lts.vehicle.VehicleConfig`
  (e.g. brake hardware, battery pack, frame construction).
- **MISSING** — config fields with no spec-sheet source (baseline kept).

The spec sheet covers roughly a dozen config fields directly (mass, wheelbase,
track, CG height, weight distribution, spring/wheel rates, motion ratio, roll
centers, steer ratio, Ackermann, torsional stiffness, differential type). The
rest of the ~80-field config has no direct CSV source and is left at the
baseline defaults — review the `TODO` and `[not in spec sheet]` markers in the
generated file before simulating.

## extract_correlation_config.py

Estimates timing offsets from a normalized replay CSV and writes a secondary
JSON config for correlation runs. The current schema includes:

- `PackPowerAdvanceS`: positive values advance `pack_voltage_v` and
  `pack_current_a` before replay sampling.
- `GpsAdvanceS`: positive values advance GPS position/course channels; the
  same value is emitted as `RawTimeOffsetS` for
  `plot_correlation_position_overlay`.

### Usage

```bash
python scripts/extract_correlation_config.py exports/correlation_lap5_replay.csv
```

Then pass the generated file into replay:

```matlab
lts.app.run_correlation( ...
    'ReplayCsv', 'exports/correlation_lap5_replay.csv', ...
    'CorrelationConfig', 'exports/correlation_lap5_correlation_config.json', ...
    'PowertrainMode', 'motor_torque_command')
```

## compare_sim_runs.py

Compares two exported simulator telemetry CSVs over the same track and prints a
compact Markdown report showing which car is stronger in low-, medium-, and
high-speed corners, acceleration, braking, top speed, and elapsed time.

The comparison is distance-aligned: both runs are interpolated onto the same
track-station grid before metrics are calculated. This avoids bias from one car
spending more time in a segment. The script uses the exported telemetry
curvature channels by default; `--track` can point to a track CSV containing
station/curvature columns, or to a `.mat` track if SciPy is installed.

### Usage

```bash
python scripts/compare_sim_runs.py exports/car_a.csv exports/car_b.csv \
    --track tracks/endurance_track_grid_25ft_from_matlab_smoothed.mat \
    --label-a R25 --label-b R26_base \
    --output exports/r25_vs_r26_report.md
```

Useful options:

| Flag | Default | Description |
|------|---------|-------------|
| `--distance-step` | `0.25` | Distance grid spacing in metres. |
| `--corner-curvature-threshold` | `0.01` | Absolute curvature threshold `[1/m]` used to identify corner samples. |
| `--throttle-threshold` | `50` | Minimum throttle `%` for acceleration-zone samples when throttle exists. |
| `--brake-threshold` | `5` | Minimum brake `%` for braking-zone samples when brake exists. |
