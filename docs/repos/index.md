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
| Who merges `staging` → `main` | — | maintainers, at release time |
| Component versions pinned in this branch of `lts` | the components' `staging` tips | the components' `main` tags |

In short: **staging pulls from staging, main pulls from main.**

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
5. At release time, the integration lead merges this repository's
   `staging` into `main` and tags.

### What is not automatic (yet)

- No bot opens "new component version available" PRs yet (Renovate is
  planned). Until then, maintainers run the checklist above.
- GitHub does **not** re-run the main tests when a component repository
  changes. The bump Pull Request in step 2–4 is what tests the combination.

The engineering record of *why* the repositories were split, the contracts
between them, and the decision log live on the
[Repository Split Plan](../repo-split/) page.
