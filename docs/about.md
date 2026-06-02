---
layout: page
title: About
permalink: /about/
---

**FSAE Transient Lap Time Simulation** is an object-oriented MATLAB framework for simulating transient-state lap times for FSAE (Formula SAE) vehicles.

## Design Philosophy

- **Swappable components** — Each subsystem (aero, suspension, powertrain, tires, track) is an abstract interface with concrete implementations. Swap models without touching the simulation core.
- **Transient simulation** — Euler integration at 1 ms timestep captures dynamic effects like pitch changes under braking and ride height variation.

## Tech Stack

- **Language:** MATLAB (R2019b+)
- **Architecture:** Strategy + Composite design patterns
- **Diagrams:** [Mermaid.js](https://mermaid.js.org/) for UML and flowchart documentation

## Links

- [Source code on GitHub](https://github.com/jyjh/lts)
- [Report an issue](https://github.com/jyjh/lts/issues)