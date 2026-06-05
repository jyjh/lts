---
layout: page
title: Data Ingestion
permalink: /data-ingestion/
---

## Data Ingestion Architecture

The current project already consumes two external data files directly through component constructors. Additional generic loader/exporter classes remain planned.

> Maintainer note: The diagram below is generated from [`data_ingestion.mmd`](data_ingestion.mmd). Edit that file, then run `node docs/sync_diagram.js` to regenerate the SVG.

![Data Ingestion Class Diagram](data_ingestion.svg)

---

### Implemented Inputs

| Data | File | Consumed By | Purpose |
|------|------|-------------|---------|
| EMRAX 228 map | `src/+components/+Powertrain/EMRAX228CC Single_4.5.mat` | `components.Powertrain.EMRAX228Powertrain` | Final-drive ratio, motor RPM curve, motor torque, and tractive force |
| Hoosier tire file | `src/+components/+Tire/43105_18x7.5_10_R25B_7.tir` | `components.Tire.TireConstants` / `PacejkaTire` | Pacejka Magic Formula coefficients and nominal tire properties |
| Test track layouts | `components.TestTrack` methods | `Simulator` / `DriverModel` | Track points, curvature, heading, and surface friction |

### Powertrain Data Behavior

`EMRAX228Powertrain` loads the MAT file and uses:

- `FDR` for total gear ratio.
- `Gearing_Map.RPM` and `Gearing_Map.Traction` for full-throttle drive-force lookup by motor RPM.
- `Speed`, `Torque`, and `Tractive_force` for compatibility torque lookup and wheel-radius inference.
- `rpmFalloffStartRPM` from the last RPM in the map.
- `rpmFalloffFactor` to shape torque falloff between the map endpoint and `rpmLimitRPM`.

### Telemetry

The current `stateLog` includes:

- Vehicle channels: time, distance, speed, acceleration, curvature, heading.
- Driver inputs: throttle, brake.
- Aero channels: downforce, drag, front/rear aero loads.
- Suspension channels: corner normal loads, damper position, damper velocity.
- Tire channels: slip ratio, wheel angular velocity, tire longitudinal/lateral force.
- Powertrain channels: drive force, motor RPM, motor torque, wheel torque, driven-wheel RPM, RPM limiter state.

### Planned Interfaces

| Class | Purpose | Status |
|-------|---------|--------|
| `TrackDataLoader` | Load GPS/cone CSV data, smooth curvature, generate racing line | Planned |
| `TelemetryExporter` | Export `stateLog` to MoTeC-compatible CSV | Planned |
| Generic aero map loader | Populate aero lookup tables from CFD data | Planned |
