---
layout: page
title: Repositories & Sync
permalink: /repos/
---

## The repository family

The simulation is split into one repository per department, plus this main
integration repository (the one you are reading the docs of) and one small
shared library. Each department works **only** in its own repository.

| Repository | What lives there | Who works there |
|---|---|---|
| `lts` (this repository) | Simulation loop, driver, vehicle configs, tracks, MoTeC tooling, correlation/governance, this documentation | Integration team |
| `lts-aero` | Aero package (mounted here at `src/+lts/+components/+Aero`) | Aero department |
| `lts-suspension` | Suspension package (`src/+lts/+components/+Suspension`) | Suspension department |
| `lts-powertrain` | Powertrain package + EMRAX motor maps (`src/+lts/+components/+Powertrain`) | Powertrain department |
| `lts-chassis` | Chassis package (`src/+lts/+components/+Chassis`) | Chassis department |
| `lts-kit` | Shared helpers used by everything (`clamp`, the gravity constant, safe `.mat` loading, ...) | Integration lead only — request changes via an issue |
| `external/MotecLogGenerator` | Third-party MoTeC `.ld` export tool | Consumed as-is, not developed here |
| `external/LTSTelemetryVisualizer` | Private team telemetry visualizer (wrapped by `scripts/visualize_correlation.m`) | Team members only — private repository, not initialized by CI or `scripts/setup.m` |

## How the main repository stays in sync (plain language)

Three facts explain the whole system:

1. **Each department repository is developed independently.** You fork it,
   make your change, and open a Pull Request into its `staging` branch.
   Its own tests run there. See
   [CONTRIBUTING](https://github.com/jyjh/lts/blob/main/CONTRIBUTING.md)
   — no git experience needed.

2. **The main repository never follows anyone automatically.** It *pins*
   (records) the exact version — a commit ID — of every component
   repository it was last tested with. Think of a bookmark that remembers
   the exact page of every book: nothing a department merges can change
   the main simulation, because main keeps reading its bookmarked pages
   until a maintainer deliberately moves a bookmark.

3. **Moving a bookmark is a reviewed, tested step.** A maintainer offers
   the newest version of a component (one command), commits that pointer
   change like any other change, opens a Pull Request, and the **full
   main-repo test suite must pass** — including the golden lap-time
   baseline, which makes any behavior change visible and deliberate.

Because of this, every clone of the main repository reproduces exactly the
same simulation, and a department can never break the main simulation by
merging something into its own repository.

### The two branches, in one table

Every repository in the family — main and components alike — has exactly
two long-lived branches:

| | `staging` | `main` |
|---|---|---|
| Purpose | All new work lands here | Stable, release-only |
| Pull Requests from forks target | `staging` | never directly |
| How it advances | PRs from forks + component bumps | **only the release cascade** (below) |
| Component versions pinned in this branch of `lts` | what the latest proven bump pinned — the components' `staging` tips after a bump PR, otherwise still the last-released `main` pins | the components' `main` heads after the cascade |

In short: **staging pulls from staging, main pulls from main.** The
component repositories never merge `staging` → `main` on their own —
their `main` branches advance only as part of a release of the main
repository, when the whole combination has been proven together.

### Releases: the cascade

A release is one operation driven from the main repository, run by the
integration lead:

```
bash scripts/release.sh            # from the main repository, staging green
```

The cascade needs a working clone of every component repository to
operate on. By default it uses **this checkout's own submodule working
copies** (`src/+lts/+util`, …) — so `git submodule update --init` is the
only preparation. To run it from a checkout without initialized
submodules, set `LTS_COMPONENTS_ROOT` to a directory holding sibling
clones (`lts-kit`, `lts-aero`, …) and those are used instead. In either
case every repository must have a clean working tree, and any local
`main`/`staging` branches must be in sync with origin.

The script does, in order:

1. **Preflight** — verifies every repository has a clean tree and that
   each `main` is an ancestor of its `staging` (so the merges below are
   safe fast-forwards). Aborts without touching anything otherwise.
2. **Components** — for `lts-kit` and each department repository:
   fast-forward `main` to the `staging` tip. *This is the moment a
   department's work officially lands on its `main` — never before.*
   (Component repositories' `.gitmodules` are branch-agnostic on both
   branches — no `branch =` lines, no per-branch flip — so the
   fast-forward leaves both branches identical — nothing to fix up.)
3. **Main repository** — merge `staging` into `main` (any submodule
   pointer conflicts resolve to the staging-proven versions), then
   restore `main`'s `.gitmodules` targeting (`branch = main`).
4. **Reconcile** — bring the release commits back into `staging` and
   restore `staging`'s targeting (`branch = staging`), so the *next*
   release is again a clean fast-forward.
5. **Push + report** — prints every branch's before/after commit and
   pushes where a remote exists.

Why this is safe: because component `main` branches only ever advance
through this cascade, the exact commits that the main repository's
`staging` branch pinned and tested become, by construction, commits on
the components' `main` branches at step 2 — the released combination is
precisely the tested combination.

**Guard rail:** CI runs `scripts/check_submodule_policy.sh` on every push
to `main`/`staging` and on every PR. It fails the build if
`.gitmodules` tracks the wrong branch for the branch being built, if a
pinned component commit does not exist on the matching component
branch, or if a component's nested `kit/` pin does not exist on
`lts-kit`'s matching branch — so a merge that would corrupt the
targeting can never land silently. Run it yourself any time with:

```
bash scripts/check_submodule_policy.sh main      # or: staging
```

### If the component folders look empty after cloning

You cloned without the pinned component versions. One command fetches
exactly the bookmarked versions:

```bash
git submodule update --init
```

Run it in the repository folder (Git Bash / terminal). After that,
`addpath('src')` in MATLAB works as documented.

### Maintainer checklist: taking a department's new version

Once a department's change is merged into their `staging`:

1. In your clone of **this** repository, update and create a branch:
   ```bash
   git checkout staging && git pull
   git submodule update --init
   git checkout -b bump/<component>
   ```
2. Offer the component's newest staging version and record it:
   ```bash
   git submodule update --remote src/+lts/+components/+Aero
   git add src/+lts/+components/+Aero
   git commit -m "chore: bump aero to latest staging"
   ```
3. Push and open a Pull Request into `staging` (yes, maintainers also use
   PRs — that is what keeps every change tested and reviewed).
4. CI must be green. If the **golden lap time** changed, that is expected
   to be the *subject of the PR discussion*, not an obstacle — say why in
   the PR description.

Releases (making a proven `staging` the new `main` everywhere) are *not*
done by hand — use the [release cascade](#releases-the-cascade) above.

### When the shared kit changes

`lts-kit` is pinned twice: by this repository (`src/+lts/+util`) and by
each component repository (its `kit/` submodule). When kit advances,
bump both layers, innermost first:

1. In each component repository that needs the new kit version, open a
   PR into its `staging` running `git submodule update --remote kit` and
   committing the pointer.
2. Then run the ordinary bump checklist above for those components —
   the main-repository bump picks up the component change and its new
   kit pin together. The CI guard checks the nested pin's containment,
   so a kit pin pointing at a commit that never reached `lts-kit`'s
   matching branch fails the build.

### What is not automatic (yet)

- No bot opens "new component version available" PRs yet (Renovate is
  planned). Until then, maintainers run the checklist above.
- GitHub does **not** re-run the main tests when a component repository
  changes. The bump Pull Request is what tests the combination, and the
  release cascade is what promotes it.

The engineering record of *why* the repositories were split, the contracts
between them, and the decision log live on the
[Repository Split Plan](../repo-split/) page.
