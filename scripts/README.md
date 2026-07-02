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

## generate_vehicle.py

Parses an **FSAE (EV) Design Spec Sheet CSV** (the format is stable year-to-year)
and emits a MATLAB vehicle file under `src/+vehicles/<Name>.m`, structured
exactly like the reference configs (`vehicles.baseline` / `VehicleConfig`).

The generated car is immediately usable: instantiate `VehicleConfig()`, then
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
| `--name`        | `generated`                      | Vehicle / function name → `src/+vehicles/<Name>.m`. Must be a valid MATLAB identifier. |
| `--driver-mass` | `68`                             | Driver mass [kg] added to the car's no-driver mass for `totalMass`. |
| `--output`      | `src/+vehicles/<Name>.m`         | Override the output path.                                          |
| `--force`       | off                              | Overwrite an existing output file (refuses otherwise).            |
| `--dry-run`     | off                              | Print the generated file and the report; write nothing.           |

Standard library only — no dependencies.

### Example

```bash
python scripts/generate_vehicle.py "2026_FSAE_Design_EV_Spec_Sheet.csv" --name R26
```

writes `src/+vehicles/R26.m`, then reference it in `run_simulation.m`:

```matlab
config = vehicles.R26();
```

### What gets mapped

The tool always prints a sourcing report to stdout with four sections:

- **DIRECT MAPPINGS APPLIED** — values written into the config (with unit
  conversions, e.g. mm→m, N/mm→N/m, N·m/deg→N·m/rad, %→fraction).
- **DERIVED (left as TODO)** — present in the CSV but ambiguous; left at the
  baseline default with the derivation sketched in a comment.
- **UNMAPPED** — present in the spec sheet but with no field in `VehicleConfig`
  (e.g. brake hardware, battery pack, frame construction).
- **MISSING** — config fields with no spec-sheet source (baseline kept).

The spec sheet covers roughly a dozen config fields directly (mass, wheelbase,
track, CG height, weight distribution, spring/wheel rates, motion ratio, roll
centers, steer ratio, Ackermann, torsional stiffness, differential type). The
rest of the ~80-field config has no direct CSV source and is left at the
baseline defaults — review the `TODO` and `[not in spec sheet]` markers in the
generated file before simulating.
