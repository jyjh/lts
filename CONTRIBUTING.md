# Contributing — no git or coding experience needed

This guide covers everything for a first contribution. If you have never
used git before, that is fine: the whole process is eight steps, and most
of them are buttons. The only tool you need beyond MATLAB is a free GitHub
account (and optionally [GitHub Desktop](https://desktop.github.com), which
turns every git step into a button).

## The idea in 30 seconds

- The code lives in **repositories**. Each department's model is its own
  repository; this one assembles them into the full simulation.
- You will **copy** the repository (a *fork*), make your change in your
  copy, then send a **Pull Request** asking the team to take it.
- Reviewers look at it, automatic tests run, a maintainer merges it.
- **You cannot break anything** by experimenting in your own copy.

## Which repository do I change?

| Your change | Where it goes |
|---|---|
| Aero, suspension, powertrain, or chassis model | That department's repository (`lts-aero`, `lts-suspension`, `lts-powertrain`, `lts-chassis`) |
| Simulation loop, driver, vehicle configs, tracks, exports | This repository |
| A shared helper (used by several repos) | `lts-kit` — open an issue here first and ask the integration lead |
| Documentation | Whichever repository holds the page you are fixing |

Every department repository has the same process as below; the only
difference is the test command (see *Running the tests*).

## Your first change, step by step

1. **Fork** (once per repository). On the repository's GitHub page, click
   *Fork* → *Create fork*. Your copy appears at
   `github.com/<your-name>/<repository>`.

2. **Get the code on your computer.** In GitHub Desktop: *File → Clone
   repository* → choose your fork. The first time you clone **this**
   repository, also fetch the department models — open a terminal (Git
   Bash) in the repository folder and run:

   ```bash
   git submodule update --init
   ```

   (If folders under `src/+lts/` look empty, this command is the fix.
   Details: [Repositories & Sync](https://jyjh.github.io/lts/repos/).)

3. **Make a branch** — a labeled copy of the code where your change
   lives. GitHub Desktop: *Branch → New branch*, name it
   `your-name/short-description`, e.g. `jsmith/stiffer-front-spring`.

4. **Make your change in MATLAB**, then run the tests (next section).
   Only move on when they are all green — the same tests will run on
   GitHub, so this saves you a round trip.

5. **Save the change.** GitHub Desktop lists your changed files under
   *Current repository → Changes*. Write a one-line summary in the
   Summary box (e.g. `suspension: stiffer front spring for R26`), then
   click **Commit to <your branch>**.

6. **Send it.** Click **Push origin**, then *Branch → Create Pull
   Request*. On GitHub, check that **base: staging** (never `main`),
   fill in the checklist, and click *Create pull request*.

7. **Watch the checks.** The orange dot next to your PR turns green
   (tests passed) or red (something failed). If red: click *Details*,
   read the **first** red line — it usually names the file and the
   problem. If it is not obvious, paste that line into your PR comment
   and ask. Asking is expected; nobody is born knowing this.

8. **Merged.** A maintainer merges your PR. Delete your branch when
   GitHub offers to — your change now lives in `staging` and will reach
   `main` at the next release.

## Running the tests

| Repository | Command (from the repository folder) |
|---|---|
| This repository (MATLAB) | `addpath('src'); addpath('scripts'); run_audit_tests` (~2.5 min; single file: `run_audit_tests('TireContactTest.m')`) |
| This repository (Python) | `python -m pip install -r requirements.txt` (once), then `python -m pytest tests -q` |
| Department repositories | In MATLAB, `cd` to the repository, then run `run_tests` (first time: `git submodule update --init --recursive`) |

Tire `.tir` data files are not committed for licensing reasons; tests that
need them skip automatically (see
`src/+lts/+components/+Tire/README.md`).

## How a change reaches `main` (releases)

You only ever interact with `staging`. When the team decides to make a
release, the integration lead runs one script (`scripts/release.sh`) in
the main repository that promotes the proven `staging` state to `main`
everywhere at once — including each department repository's `main`. This
is the only way anything reaches a `main` branch, which is why `main` is
always a combination that passed the full test suite. You never need to
do this yourself; see
[Repositories & Sync](https://jyjh.github.io/lts/repos/) if you are
curious how it works.

## Rules of thumb

- SI units everywhere (m, kg, N, s, rad); state units where numbers are
  not obvious.
- Comments explain *why*, not *what*.
- Never commit TTC tire data or private team logs.
- Files over 5 MB: open an issue and ask first.
- Never edit the folders `src/+lts/+util` or `src/+lts/+components/+Aero`
  (etc.) inside **this** repository — that is other repositories' code,
  mounted here read-only. Send your change to the department repository.

## If something goes wrong

| You see | What to do |
|---|---|
| Folders under `src/+lts/` are empty | Run `git submodule update --init` in the repository folder |
| A test fails on your computer | Read the first error line it prints; if unclear, open the test file named in the message, or ask |
| Red ✗ on your Pull Request | *Details* → first red line; paste it into the PR and ask if stuck |
| "Merge conflict" (someone changed the same lines) | Tell a maintainer in your PR — conflicts are normal and fixable; trying random buttons is the only wrong move |
| You changed the wrong repository | Nothing is broken — tell a maintainer in your PR |
| GitHub asks about "2FA" or permissions | Ask a maintainer; do not work around it |

## Git words you will see

- **fork** — your personal copy of a repository on GitHub
- **clone** — download a repository to your computer
- **branch** — a labeled copy of the code for one change
- **commit** — save a snapshot of your change on your computer
- **push** — upload your commits to your fork on GitHub
- **pull request (PR)** — asking the team to take your change
- **merge** — the team accepting it
- **CI** — the automatic tests GitHub runs on every PR

## Where to ask

Open an issue (any template, choose what fits) in the relevant repository,
or contact the maintainer listed in that repository's README ownership
table. How the repositories fit together and stay in sync:
[Repositories & Sync](https://jyjh.github.io/lts/repos/).
