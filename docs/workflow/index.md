---
layout: page
title: Workflow
permalink: /workflow/
---

## Department Workflow

The following diagram shows how data flows from each engineering department through to simulation and back for iterative improvement:

> **Maintainer note:** The diagram below is generated from [`workflow.mmd`](workflow.mmd). Edit that file, then run `node docs/sync_diagram.js` to regenerate the SVG.

![Department Workflow Diagram](workflow.svg)

---

### How it works

1. **Aero Department** — CFD simulations produce aero coefficient maps, which are implemented as `AeroComponent` subclasses (e.g., `FrontWing`, `UnderbodyFloor`).
2. **Suspension Department** — Kinematics analysis produces roll center and camber gain data, implemented as `SuspensionComponent` subclasses.
3. **Powertrain Department** — Dyno testing produces torque/power lookup tables, implemented as `PowertrainComponent` subclasses.
4. **Vehicle Dynamics** — All component objects are instantiated and assembled in `VehicleManager`. GPS track data is loaded into a `Track` subclass.
5. **Simulation** — `VehicleManager.simulate()` runs the lap and exports telemetry as MoTeC-compatible CSV.
6. **Validation** — Simulated telemetry is compared against real MoTeC data. Discrepancies drive iterative improvements back to each department.