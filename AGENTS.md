# AGENTS.md — mandatory git workflow for agentic workers

This file is the operating contract for any AI agent (Claude Code, Codex,
ZCode, etc.) working in this repository family. It condenses
[CONTRIBUTING.md](CONTRIBUTING.md) and
[Repositories & Sync](docs/repos/index.md) into rules an agent must follow
without being asked. When this file and the canonical docs disagree, the
canonical docs win — fix this file in the same PR.

## The model in five sentences

1. This repository (`lts`) is the **integration** repo; each department model
   (aero, suspension, powertrain, chassis) and the shared kit live in their
   **own repositories**, mounted here as submodules at their original
   `src/+lts/...` paths.
2. Every repository has exactly two long-lived branches: `staging` (where all
   new work lands) and `main` (release-only).
3. Work lands on `staging` **only through pull requests** — CI rejects any PR
   targeting `main`, and branch protection forbids direct pushes to both
   branches.
4. `main` advances **only** via `scripts/release.sh` (the release cascade);
   no agent ever hand-merges `staging` into `main`.
5. The integration repo pins an exact commit of every component; moving a
   pin is itself a reviewed, CI-tested PR.

## Where a change goes

| You are changing | It belongs in | Path there (mounted here) |
|---|---|---|
| Simulation loop, driver, vehicles, tracks, correlation, governance, docs, CI | this repo | — |
| Aero / Suspension / Powertrain / Chassis component code | that department's repo (`lts-aero`, …) | `src/+lts/+components/+Aero`, … |
| Shared helpers (`clamp`, constants, `.mat` loading) | `lts-kit` (integration-lead approval; open an issue first) | `src/+lts/+util` |
| Third-party MoTeC tooling | nowhere — consumed as-is | `external/MotecLogGenerator` |

**Never edit files under `src/+lts/+util` or `src/+lts/+components/*` inside
this repository.** Those folders are other repositories' code mounted here
read-only; edits there are silently lost on the next submodule update. Make
the change in the component repository instead (same workflow as below).

## The standard work loop (this repo)

Run this from the repository root (Git Bash):

```bash
git checkout staging && git pull            # start from proven staging
git submodule update --init                 # folders under src/+lts/ must never be empty
git checkout -b <area>/<short-topic>        # e.g. correlation/my-fix, docs/my-page
# ... make changes ...
# run the test suites (below) — they must be green before you commit
git add <files> && git commit
git push -u origin <area>/<short-topic>
gh pr create --base staging                 # NEVER --base main
```

Branch naming follows `<area>/<short-topic>` (existing examples:
`correlation/segmented-replay`, `fidelity/brake-and-slip-accuracy`,
`ci/pr-target-guard`). Component-pin branches are `bump/<component>`.

Before pushing, re-check: you are **not** on `main` or `staging`, the commit
contains **no** forbidden files (below), and the tests passed.

## Tests that must pass before you claim done

| Repo | Command |
|---|---|
| this repo (MATLAB) | `addpath('src'); addpath('scripts'); run_audit_tests` (~2.5 min) |
| this repo (Python) | `python -m pytest tests -q` |
| component repos | `cd` into the repo, run `run_tests` (needs `git submodule update --init --recursive` once) |

Tire `.tir` data files are untracked for licensing reasons; tests needing
them skip automatically. A changed **golden lap time** is not a failure —
it must be the subject of the PR description, with the physical reason.

## Working in a component repository

The same model applies there: fork/clone the component repo, branch from its
`staging`, change, run its `run_tests`, PR into its **`staging`**. Then,
because component repos use **squash merges**, the landed commit has a new
SHA — record it here with a pin-bump PR:

```bash
git checkout staging && git pull && git checkout -b bump/<component>
git submodule update --remote src/+lts/+components/+Aero   # (or the component you landed)
git add src/+lts/+components/+Aero
git commit -m "chore: bump aero to latest staging"
# push, PR into staging — the full main-repo suite gates the new combination
```

Submodule targeting is per-branch and CI-enforced
(`scripts/check_submodule_policy.sh`): on `staging`, `.gitmodules` tracks
each component's `staging` branch; on `main`, its `main` branch. Pins must
exist on the tracked branch, including each component's nested `kit/` pin.
Never flip `branch =` lines yourself; the release script owns that.
`lts-kit` is pinned twice (here and inside each component) — bump the
component's inner `kit/` pin first, then the component pin here.

## Releases

Never promote `staging` → `main` by hand, in any repository of the family.
Releases are one command, run by the integration lead from this repo with
every working tree clean and local `main`/`staging` synced with origin:

```bash
bash scripts/release.sh
```

If a task looks like "merge this to main" or "publish a release", stop and
say so instead — the cascade is a human-owned operation.

## Hard rules

- **PRs target `staging`, never `main`** (CI hard-fails otherwise).
- **No direct pushes to `main` or `staging`** in any repository.
- **Never commit TTC `.tir` tire data or private team logs** (licensing).
  Files over 5 MB: open an issue and ask before committing.
- **Do not edit** `.gitmodules`, `scripts/release.sh`,
  `scripts/check_submodule_policy.sh`, `.github/workflows/ci.yml`, or
  `CODEOWNERS` without integration-lead review — CODEOWNERS routes them.
- Contract changes (cfg schemas, telemetry channel names) follow the two-PR
  procedure in [Component Contracts](docs/contracts/index.md) — ask first.
- SI units everywhere; comments explain *why*, not *what*.
- Match the repo's commit style: `area: imperative summary`, e.g.
  `feat(correlation): ...`, `docs: ...`, `ci: ...`, `chore(release): ...`.
- Leave every working tree you touched clean: commit or stash deliberately,
  and never end a session with a dirty submodule or a detached-HEAD
  component holding uncommitted edits.

## Before finishing any task — self-check

1. `git status` is clean in the parent repo **and** in every submodule you
   touched (`git submodule status` shows no `+`/`-` markers).
2. Work is committed on a `<area>/<topic>` branch created from `staging`,
   not on `main`/`staging`/a detached HEAD.
3. Both test suites ran green on the final code state.
4. The PR (if opened) targets `staging` and its description explains any
   golden-lap-time change.

## Canonical references

- [CONTRIBUTING.md](CONTRIBUTING.md) — the human walkthrough of the same loop
- [Repositories & Sync](docs/repos/index.md) — pins, bumps, cascade, CI guard
- [Repository Split Plan](docs/repo-split/index.md) — why the family looks
  like this; the decision log
- [Component Contracts](docs/contracts/index.md) — cfg/telemetry interfaces
- [workflow.md](workflow.md) / [setup.md](setup.md) — the engineering
  method (calibrate once, predict the change, never recalibrate the variant)
