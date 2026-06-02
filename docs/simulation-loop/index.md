---
layout: page
title: Simulation Loop
permalink: /simulation-loop/
---

## Simulation Loop Sequence

The following sequence diagram shows how `VehicleManager.simulate()` orchestrates the simulation each timestep:

> **Maintainer note:** The diagram below is generated from [`simulation_loop.mmd`](simulation_loop.mmd). Edit that file, then run `node docs/sync_diagram.js` to regenerate the SVG.

![Simulation Loop Sequence Diagram](simulation_loop.svg)

---

### Step-by-step walkthrough

1. **Entry** — `App` calls `VM.simulate()` to start the lap.
2. **State read** — Current vehicle state (speed, accelerations, pitch, position) is read.
3. **Driver model** — Look-ahead along the track determines throttle and brake inputs.
4. **Aero computation** — Each aero component computes downforce/drag based on current pitch and ride height.
5. **Load transfer** — Suspension computes lateral and longitudinal load transfers.
6. **Tire forces** — Tire model computes combined lateral and longitudinal forces at the grip limit.
7. **Force resolution** — All forces and yaw moments are summed.
8. **State update** — Vehicle state is integrated forward one timestep using Euler integration.
9. **Logging** — Current state is appended to the history log.
10. **Completion** — When the vehicle exits the track, the full `stateLog` and `lapTime` are returned.