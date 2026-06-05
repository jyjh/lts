---
layout: page
title: About
permalink: /about/
---

**FSAE Transient Lap Time Simulation** is an object-oriented MATLAB framework for simulating transient-state lap times for FSAE vehicles.

## Design Philosophy

- **Swappable components** - Aero, suspension, powertrain, tire, and track behavior live behind component classes so models can be replaced without rewriting the simulation loop.
- **Transient state tracking** - Vehicle, suspension, tire, and powertrain state objects persist across timesteps and carry the dynamic quantities needed by each subsystem.
- **Data-backed components** - The current EMRAX 228 model reads a provided `.mat` tractive-force map, and the Pacejka tire model reads a provided `.tir` file.
- **Focused telemetry** - `stateLog` captures speed, acceleration, aero loads, suspension travel, tire loads, tire slip/forces, motor RPM, torque, and RPM limiter state for plotting and debugging.

## Tech Stack

- **Language:** MATLAB
- **Tire model:** MFeval Pacejka Magic Formula evaluation
- **Diagrams:** Mermaid.js source diagrams rendered to SVG

## Links

- [Source code on GitHub](https://github.com/jyjh/lts)
- [Report an issue](https://github.com/jyjh/lts/issues)
