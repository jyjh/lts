# Tire data files (`.tir`) are intentionally NOT tracked

This folder has no `.tir` files in a fresh clone. The tire property files used
by the vehicle configs and tests are:

| File | Used by |
|---|---|
| `43105_18x7.5_10_R25B_7.tir` | `baseline`, `R26_base`, `R25` (default tire), most tests |
| `43105_18x7.5_10_R25B_7_theoretical.tir` | `weight_savings_*` analysis scripts (default) |
| `Hoosier 43100 18.0x6.0-10 R20_7.tir` | R20 reference data |
| `Hoosier 43100 18.0x6.0-10 R20_7 - Scaled.tir` | `R25_correlation_tuning` overlay, tire tests |

## Why they are untracked

These files are fitted from **FSAE Tire Test Consortium (TTC)** rig data,
which is distributed to member teams under terms that do not permit public
redistribution. Committing them to a public repository would redistribute
member-only data, so they were removed from version control (they remain on
the original authors' machines). The same applies to derived/edited copies —
`magicformula2.tir` (an unreferenced leftover, a lightly edited copy of the
Hoosier 43100 file) is untracked for the same reason and can be deleted
locally.

## How to restore them

1. Get the fitted `.tir` files from the team's data store (they were fitted
   from TTC data under the team's membership — ask the team's TTC account
   holder).
2. Place the four files listed above in this folder
   (`src/+lts/+components/+Tire/`).
3. Tire-dependent tests auto-skip when the files are absent (see
   `tests/tireDataAvailable.m`); with the files present they run normally.

Note: the files still exist in this repository's git **history** (they were
tracked before this change). Cloning the repo today still gives access to the
historical blobs. Removing them entirely requires rewriting published history
— see "Purging the files from git history" below.

## Purging the files from git history

The `.tir` blobs live in ~11 commits on the published `main` branch (and
possibly on other remote branches). A full purge rewrites every commit SHA on
every branch, so it is a coordinated, force-push operation — not part of the
normal untracking change. When you are ready to do it:

1. Land/merge all pending work first (rewriting history with uncommitted
   changes in flight is asking for pain).
2. `python -m pip install git-filter-repo`
3. Work in a **fresh clone** (filter-repo refuses dirty checkouts):
   ```bash
   git clone https://github.com/jyjh/lts.git lts-purge && cd lts-purge
   git filter-repo --invert-paths --path-glob '*.tir'
   ```
4. filter-repo removes the `origin` remote as a safety measure; re-add it,
   then force-push **all** branches and tags:
   ```bash
   git remote add origin https://github.com/jyjh/lts.git
   git push origin --force --all
   git push origin --force --tags
   ```
5. Delete stale remote branches that still point at old history (e.g. the
   `devin/*` and `codex` branches) — otherwise the old commits, tire blobs
   included, remain reachable on GitHub.
6. Everyone with a clone must re-clone or hard-reset onto the rewritten
   branches. Restore the local `.tir` files afterwards (they are gitignored
   now, so they will not come back by accident).
7. Caveat: GitHub can serve already-forked or cached copies of the old
   commits for some time (and any forks made before the purge keep the
   blobs). For a truly complete takedown, contact GitHub support to request
   garbage collection of the dangling objects and check for forks.

