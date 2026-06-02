---
layout: page
title: Data Ingestion
permalink: /data-ingestion/
---

## Data Ingestion Architecture

The following class diagram shows the planned interfaces for loading external data into the simulation and exporting telemetry:

> **Maintainer note:** The diagram below is generated from [`data_ingestion.mmd`](data_ingestion.mmd). Edit that file, then run `node docs/sync_diagram.js` to regenerate the SVG.

![Data Ingestion Class Diagram](data_ingestion.svg)

---

### Component Overview

| Class | Purpose | Status |
|-------|---------|--------|
| `ExternalDataLoader` | Abstract interface for all file I/O | Planned |
| `AeroMapLoader` | Parses CFD aero coefficient maps and interpolates downforce by ride height | Planned |
| `TrackDataLoader` | Loads GPS/cone CSV data, smooths curvature, generates racing line | Planned |
| `TelemetryExporter` | Exports `stateLog` to MoTeC-compatible CSV format | Planned |

### Data Flow

1. **Aero data** — `AeroMapLoader` reads CFD output CSVs and populates `SimpleAero` or custom `AeroComponent` lookup tables.
2. **Track data** — `TrackDataLoader` parses GPS coordinates, smooths the curvature profile, and builds `TestTrack` points.
3. **Telemetry export** — After simulation, `VehicleManager` sends the `stateLog` to `TelemetryExporter` which writes a MoTeC-compatible CSV for comparison with real data.