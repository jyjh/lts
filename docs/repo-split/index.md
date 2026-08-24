---
layout: page
title: Repository Split Plan
permalink: /repo-split/
---

## Repository Split Plan

**Status:** split executed locally on 2026-08-24 (see [decision log](#decision-log));
pending: push of the six repositories and transfer to the organization.
**Owner:** simulation lead (rotate per the ownership table below).
**Last updated:** 2026-08-24.

This page is the master plan for moving the department component models
(Aero, Suspension, Powertrain, Chassis) out of this monorepo into their own
repositories under a team organization, with this repo (`lts`) as the
integration main repo. It is written to be self-contained: a member joining
after the split should be able to understand what was done and why from this
page alone. Nothing here is executed until the open decisions in
[Decisions](#decisions) are closed.

### Why split

- **Department focus.** Each subsystem team works in its own small repository
  with its own issues, CI, and release tags, instead of touching a monorepo
  whose CI they don't own.
- **Org transfer.** Repositories move to a team organization with per-team
  permissions; ownership follows departments.
- **Explicit contracts.** Cross-repo usability is enforced by CI (conformance
  tests + integration tests) instead of by convention.

**Non-goals:** changing the physics, changing the MATLAB package layout
(`+lts/+components/...` paths stay identical), or splitting `tracks/`,
`config/`, or the governance/correlation tooling — those stay in main.

### Measured current state

Audit of 2026-08-24 (before the Phase 0 refactor below):

| Package | Lines | Cross-package name references after Phase 0 |
|---|---|---|
| `+lts/+components/+Aero` | ~430 | none |
| `+lts/+components/+Chassis` | ~830 | **none** (was 6 `isa` checks on `SuspensionManager`) |
| `+lts/+components/+Suspension` | ~2020 | **none** (was 1 `isa` check on `ChassisComponent`) |
| `+lts/+components/+Powertrain` | ~1670 | none |
| `+lts/+components/+Tire` | ~1180 | none |

Other facts that shape the plan:

- `lts.util` (12 small functions: `clamp`, `saturate`, `fieldOr`,
  `loadMatSafe`, ...) is called from every component package. It is the shared
  kernel and becomes its own repo.
- The composition root — `lts.vehicle.VehicleManager.fromConfig`,
  `lts.simulation.Simulator`, `lts.driver.DriverModel` — stays in main; it is
  the only place that knows all packages simultaneously.
- History is small (137 commits, ~13 MiB pack), so per-component history
  extraction with `git filter-repo` is cheap.
- Two submodules already exist (`external/MotecLogGenerator` public,
  `external/LTSTelemetryVisualizer` private) and CI already does selective
  submodule init — the pattern is proven in this repo.
- TTC `.tir` tire data files are untracked on purpose (redistribution
  restrictions); any future Tire repo must keep that policy.

### Target repository map

| Repository | Contents | Mount point in main |
|---|---|---|
| `org/lts` (this repo) | Simulator, driver, app entry points, `+vehicles`, telemetry, correlation/governance/prediction, tracks, integration tests, this docs site | — |
| `org/lts-kit` | `+lts/+util/*` shared kernel | `src/+lts/+util` |
| `org/lts-aero` | `+lts/+components/+Aero` | `src/+lts/+components/+Aero` |
| `org/lts-suspension` | `+lts/+components/+Suspension` | `src/+lts/+components/+Suspension` |
| `org/lts-powertrain` | `+lts/+components/+Powertrain` + EMRAX `.mat` maps | `src/+lts/+components/+Powertrain` |
| `org/lts-chassis` | `+lts/+components/+Chassis` | `src/+lts/+components/+Chassis` |

Each component repository contains its package files **at the repository
root** (a submodule's working tree is its root, so the mounted content is
exactly the old package folder). Each mounts `lts-kit` as a submodule at
`kit/` and ships `run_tests.m`, which assembles a temporary `+lts` package
sandbox in `build/` (gitignored) — the repository's classes plus kit's
`+util` — so the package runs standalone without installing anything into
a parent repository. Because MATLAB resolves packages from the parent of
`+lts`, mounting at today's exact paths means **zero source-file changes
in main**: `addpath('src')` keeps working.

`+Tire` stays in main for now: it changes rarely, is entangled with
correlation work, and its data files can't be committed anyway. The same
recipe applies later if the tire group wants ownership.

### Decisions

Closed decisions are recorded in the [decision log](#decision-log). Still
open — close these before Phase 1:

1. **Public vs private organization repos.** Free MATLAB on GitHub Actions
   runners requires *public* repositories; our CI already relies on it.
   Private repos need a license server or self-hosted runners. Recommended:
   component repos public (model code only, no team data), main repo and
   telemetry private if needed, with a fine-grained PAT (`contents:read`)
   secret for submodule init in CI.
2. **`lts-kit` change policy.** Proposed: integration lead approves all kit
   changes; semver tags; treated like a published library. If kit churn is
   high, the package boundaries are wrong — revisit the split before growing
   the kit.
3. **Organization name and team structure** (`aero`, `suspension`,
   `powertrain`, `chassis`, `integration` teams).

### The cross-repo contract

Three things form the interface between a component repo and main. Each must
be explicit and tested:

1. **Abstract base classes** (`AeroComponent`, `ChassisComponent`,
   `SuspensionManager` structural interface, `PowertrainComponent`,
   `TireModel`) — live with their component repo; main consumes them via the
   submodule.
2. **The `cfg` struct schema** (`cfg.suspension.*`, `cfg.chassis.*`, ...).
   Each component repo ships a `validateConfig(cfg)` function plus a
   documented schema page (fields, units, valid ranges). This is the formal
   channel through which department data reaches main and dovetails with the
   governed-prediction parameter roles (`design` vs `fixed_measured` — those
   parameters cannot be fitted).
3. **Telemetry channel names** the Simulator logs. Each component repo keeps
   a conformance test that pins its channel names/units so a department
   cannot silently rename `Damper Travel FL (mm)` and break the MoTeC
   exporter.

### Phases

#### Phase 0 — prepare inside the monorepo (current phase)

- [x] Decouple Chassis ↔ Suspension: all class-name `isa` checks replaced
      with structural capability checks (`isprop`/`ismethod`). The two
      packages no longer reference each other by name (2026-08-24, full
      280-test suite green).
- [x] Gravity moved to the shared kernel (`lts.util.PhysicalConstants`);
      `VehicleManager.g` delegates, `SimpleChassis` reads it directly
      (2026-08-24).
- [x] EMRAX `.mat` maps moved into the `+Powertrain` package folder and
      resolved relative to it, so the powertrain repository is
      self-contained (2026-08-24).
- [ ] Per-component `validateConfig(cfg)` + schema docs (contract item 2).
- [ ] Per-component telemetry-channel conformance tests (contract item 3);
      the current smoke tests pin constructor/return shapes only.
- [x] Tag `pre-split`-equivalent anchor: commit `4db2aef` is the last
      monorepo commit before the submodule rewiring.

#### Phase 1 — organization setup

- [ ] Close the open decisions above.
- [ ] Create the org, teams, and the six empty repos.
- [ ] Create the org-level `.github` repository with shared
      `CONTRIBUTING.md`, PR/issue templates, and `HANDOVER.md` (until then,
      the templates in this repo's `.github/` are the source of truth).

#### Phase 2 — extract histories

**Executed 2026-08-24** with `git filter-repo`; all 40 extracted files
verified byte-identical to the monorepo by blob hash. Each repository
carries its package's full history and a harness commit (tests, CI,
README, CONTRIBUTING, LICENSE, `lts-kit` submodule at `kit/`,
`run_tests.m` sandbox runner). The reference procedure, kept for future
extractions (e.g. Tire):

```bash
git clone https://github.com/org/lts lts-aero-tmp
cd lts-aero-tmp
git filter-repo --path src/+lts/+components/+Aero/ \
                --path-rename src/+lts/+components/+Aero/:+lts/+components/+Aero/
git remote add origin https://github.com/org/lts-aero.git
git push -u origin main
```

Then add, as ordinary commits on top: `tests/` (moved from main),
`run_tests.m`, `.github/workflows/ci.yml`, `README.md`, `CONTRIBUTING.md`
(from the org `.github` repo), `LICENSE` (MIT, same as main), and the
`lts-kit` submodule mounted at `kit/`.

#### Phase 3 — rewire main

**Executed 2026-08-24.** The five packages were `git rm`'d and re-added as
submodules with clean names (`kit`, `aero`, `suspension`, `powertrain`,
`chassis`), relative URLs (`../lts-aero` — org-relative once transferred),
and `branch = main` entries. Both full suites (MATLAB 280, pytest 30) pass
against the mounted submodules. For each extracted package: `git rm -r` the
folder, then

```bash
git submodule add https://github.com/org/lts-aero.git src/+lts/+components/+Aero
git submodule add https://github.com/org/lts-kit.git      src/+lts/+util
```

- Use **relative URLs** in `.gitmodules` (`url = ../lts-aero.git`) so an org
  rename doesn't break clones.
- Verify each mount is byte-identical to the `pre-split` tag (diff must be
  empty) — this is the acceptance test for the whole phase.
- Move unit tests to their component repos
  (`ChassisLoadTransferTest` → chassis, `SuspensionQuarterCarTest` →
  suspension, `PowertrainDifferentialTest` → powertrain). Everything that
  references `Simulator`, correlation, or governance stays in main.
- Add a one-command bootstrap (`scripts/setup.m`) that initializes
  submodules and prints the `addpath` lines, so nobody needs to know
  submodule commands.

#### Phase 4 — transfer and links

- Transfer `jyjh/lts` to the org (repo URLs redirect; **GitHub Pages URLs do
  not** — update every `jyjh.github.io/lts/...` link in README and docs).
- Update `.gitmodules` URLs and README prose references.

#### Phase 5 — process freeze

- Branch protection on every repo: CI green + 1 approval, no direct pushes to
  `main`, squash-merge, auto-delete branches.
- CODEOWNERS per department; `.gitmodules` changes require integration-lead
  review.
- Enable Renovate in main with `git-submodules` enabled: component repos tag
  semver releases, Renovate opens "update lts-aero to v1.3.0" PRs, main's CI
  validates the bump, integration lead merges. Nobody hand-edits SHA
  pointers.

#### Phase 6 — documentation and handover

- Update the [Department Workflow](../workflow/) page to the new repo map.
- Publish each repo's schema doc; link from the docs site.
- First ADRs already recorded (below); record the split itself as an ADR.
- Record the inaugural handover walkthrough.

### CI design

Principle: **a component repo's CI proves the component works standalone;
main's CI proves the pinned combination works together.** Neither trusts the
other.

- Component CI (public repo, free MATLAB runner, pinned R2026a):
  `checkout` with submodules → `setup-matlab` → `addpath(pwd)` → run the
  component's unit tests + contract conformance tests.
- Main CI: today's two jobs (Python + MATLAB) plus recursive submodule init
  (PAT-substituted URL config for private submodules) and the
  `GoldenLapTimeTest` as the headline integration gate — a component bump
  that shifts golden lap time is exactly the review conversation to force.

### Contribution model for department members

Written for contributors who are new to git. The full loop is deliberately
five steps (see each repo's `CONTRIBUTING.md`): branch → change → run
`run_tests.m` → push → open a Pull Request with the checklist filled in.
Rules that matter: never push to `main`, never edit `+lts/+util/`, ask before
committing data files over 5 MB, SI units everywhere. The PR/issue templates
in this repo's `.github/` folder are the current source of truth and are
lifted to the org `.github` repo at Phase 1.

Handover sustainability (multi-batch): ADR-style decision log (below),
ownership table in each README (department, maintainer handle, term),
`HANDOVER.md` checklist in the org `.github` repo, semver tags + short
CHANGELOG per component repo as each term's "seal".

### Risks

| Risk | Mitigation |
|---|---|
| Submodule friction lands on one person | Integration-lead role is named in every ownership table and rotated every term; Renovate automates bumps |
| Departments drift from the contract | Conformance tests in component CI + golden lap time in main CI |
| Private-repo MATLAB licensing surprise | Decide visibility before Phase 1; component repos public |
| Kit becomes a dumping ground | Kit changes need integration-lead approval; high churn = revisit boundaries |
| Docs silos after split | This docs site remains the single index; component repos link here, not the reverse |

### Decision log

- **2026-08-24 — ADR: Chassis and Suspension decoupled structurally.**
  `SimpleChassis` now gates suspension access on structural capability
  checks (`providesSuspensionInterface`: corner-unit, roll-center, and
  anti-geometry properties plus `getAxleRollStiffness`), cached via
  `setSuspension` exactly as before; `SuspensionManager` gates its optional
  chassis-roll coupling on `ismethod` probes for `getFrontRollAngle` /
  `getRearRollAngle`. No behavior change; full 280-test suite green. This
  makes a future Chassis/Suspension two-repo split possible without either
  repo requiring the other on the MATLAB path.
- **2026-08-24 — Plan adopted:** split into `lts` (integration) + `lts-kit`
  + per-department component repos, submodules mounted at today's paths,
  contracts enforced by conformance + integration CI. Alternative rejected:
  single monorepo with CODEOWNERS-protected department folders (would give
  focus without submodule mechanics, but not per-repo permissions,
  ownership, or independent versioning under the org).
- **2026-08-24 — ADR: Split executed locally.** Five repositories
  (`lts-kit`, `lts-aero`, `lts-suspension`, `lts-powertrain`,
  `lts-chassis`) extracted with `git filter-repo` from commit `4db2aef`
  and mounted in main at their original paths; histories preserved,
  40 files blob-hash-verified identical. Component repositories carry a
  standalone harness: `run_tests.m` builds a `+lts` sandbox in `build/`
  (repo root = package content; kit nested at `kit/`). All five standalone
  suites and both main suites green. Pending: push + org transfer
  (relative `../lts-*` URLs already resolve org-relative).
- **2026-08-24 — ADR: main/staging dual-branch model with fork-only
  development.** Every repository (main and components) has exactly two
  long-lived branches: `staging` (all PRs from forks land here; submodule
  pointers track component `staging` branches via per-branch `.gitmodules`
  `branch` entries) and `main` (release-only; pointers track component
  `main`/tags; merged from `staging` by maintainers). No direct pushes;
  all work from forks. Previous stale branches (`codex`, `matlab-ci`,
  `reorientation`, `temp`, `devin/*`) were deleted after merge-status
  review; their tips were `a88e498`, `5b114d6`, `a93160c`, `90c113f`,
  `142ac29`, `b764e61`, `03bef3a`, `ee560ba` respectively.
