# FSAE Transient Lap Time Simulation

An object-oriented MATLAB lap-time simulation for FSAE vehicles. This
repository — **lts**, the *main integration repository* — assembles the
department component repositories into a complete vehicle and runs the
transient simulation loop through `lts.simulation.Simulator`. Each
department's model lives in its own repository; this page only tells you
where to go.

## Quick start

```bash
git clone --recurse-submodules <this repository>
```

```matlab
addpath('src')
lts.app.run_simulation          % writes exports/motec_<track>_<car>_<time>.csv/.ld
```

Full environment setup (MATLAB version, tire data, Python tooling):
[setup.md](setup.md) and [Requirements](https://jyjh.github.io/lts/#requirements).

## Where to go

| You want to... | Go to |
|---|---|
| Set up the environment and run your first simulation | [setup.md](setup.md) |
| Take department data to a design decision | [workflow.md](workflow.md) · [Department Workflow](https://jyjh.github.io/lts/workflow/) |
| Change the aero / suspension / powertrain / chassis model | Your department's own repository (`lts-aero`, `lts-suspension`, `lts-powertrain`, `lts-chassis`) — overview: [Repositories & Sync](https://jyjh.github.io/lts/repos/) |
| Add or change shared helper code | `lts-kit` — ask the integration lead first ([Repositories & Sync](https://jyjh.github.io/lts/repos/)) |
| Contribute code or report a problem | [CONTRIBUTING.md](CONTRIBUTING.md) — written for first-time contributors, no git experience needed |
| Understand how the repositories stay in sync | [Repositories & Sync](https://jyjh.github.io/lts/repos/) |
| Understand the simulation itself | [Architecture & Usage](https://jyjh.github.io/lts/) · [Simulation Loop](https://jyjh.github.io/lts/simulation-loop/) · [Physics Flow](https://jyjh.github.io/lts/physics-flow/) · [Class Diagram](https://jyjh.github.io/lts/class-diagram/) |
| Work with real MoTeC logs (replay, tuning) | [Correlation Replay](https://jyjh.github.io/lts/correlation/) |
| Make a governed design prediction | [Governed Prediction](https://jyjh.github.io/lts/governed-prediction/) |
| Add or update a track | [Tracks](https://jyjh.github.io/lts/tracks/) |
| See how data flows into the models | [Data Ingestion](https://jyjh.github.io/lts/data-ingestion/) |
| Why the repositories were split — engineering record and contracts | [Repository Split Plan](https://jyjh.github.io/lts/repo-split/) |

The component repositories are mounted inside this one at their original
paths (`src/+lts/+util`, `src/+lts/+components/+Aero`, ...), so `addpath('src')`
works unchanged — you never edit those folders here; see
[Repositories & Sync](https://jyjh.github.io/lts/repos/).

## Documentation site

All guides live at [jyjh.github.io/lts](https://jyjh.github.io/lts).

## License

MIT — see [LICENSE](LICENSE).
