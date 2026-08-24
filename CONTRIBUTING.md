# Contributing to lts (main integration repository)

This repository integrates the department component repositories and owns
the simulation loop, driver, vehicles, tracks, correlation/governance
tooling, and documentation. All development happens on **forks**; `main`
and `staging` are protected.

## The loop (fork-based)

1. **Fork** this repository once (GitHub → *Fork*, top right).
2. Clone your fork with submodules:

   ```
   git clone --recurse-submodules <your-fork-URL>
   ```

3. Branch from `staging`: `git checkout staging && git checkout -b your-name/short-description`
4. Make your change. Run the tests:

   ```
   MATLAB:   addpath('src'); addpath('scripts'); run_audit_tests
   Python:   python -m pytest tests -q
   ```

5. Push to your fork and open a **Pull Request targeting `staging`**,
   filling in the PR checklist.
6. A maintainer reviews; CI must be green; they merge.

## Branch model

- `staging` — integration branch. All PRs from forks target it. Its
  submodule pointers track the component repositories' `staging` branches.
- `main` — stable and release-only. Maintainers merge `staging` → `main`
  after integration tests (including the golden lap-time baseline) pass,
  and tag. Its submodule pointers only move to component `main` heads or
  version tags.
- **Staging pulls from staging, main pulls from main**: each branch of this
  repository pins each component submodule to the matching branch of the
  component repository. Maintainers refresh pointers with:

  ```
  git checkout staging
  git submodule update --remote
  git add src/+lts/+util src/+lts/+components/+Aero \
          src/+lts/+components/+Suspension src/+lts/+components/+Powertrain \
          src/+lts/+components/+Chassis
  ```

## Where component work happens

The department packages live in their own repositories — fork and PR there
for model changes; each has the same branch model and a standalone
`run_tests`:

| Repository | Mounted at | Department |
|---|---|---|
| `lts-kit` | `src/+lts/+util` | Integration (shared kernel) |
| `lts-aero` | `src/+lts/+components/+Aero` | Aero |
| `lts-suspension` | `src/+lts/+components/+Suspension` | Suspension |
| `lts-powertrain` | `src/+lts/+components/+Powertrain` | Powertrain |
| `lts-chassis` | `src/+lts/+components/+Chassis` | Chassis |

You normally never touch these directories inside this repository except
as a maintainer bumping pointers (see above).

## Rules

- Never commit directly to `main` or `staging` here — always a fork PR.
- SI units everywhere; comments explain *why*, not *what*.
- Never commit TTC tire `.tir` data or team telemetry beyond the tracked
  examples; see `src/+lts/+components/+Tire/README.md` and the dataset
  catalog.
- Data files over 5 MB: open an issue and ask first.
- If CI is red on your PR: open the failing check, read the first error
  line — it is usually the answer. If not, paste it into the PR and ask.

Issue/PR templates are in [`.github/`](.github/). Background and the
repository-split contract: <https://jyjh.github.io/lts/repo-split/>.
