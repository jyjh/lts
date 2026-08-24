#!/usr/bin/env bash
# check_submodule_policy.sh — verify the main repository's submodule policy.
#
# Policy (see the Repositories & Sync documentation page):
#   1. On branch `main`, .gitmodules tracks each lts-* submodule's `main`
#      branch; on branch `staging`, it tracks `staging`. A merge must never
#      leave the wrong value behind.
#   2. The pinned commit (gitlink) of each lts-* submodule must exist on
#      the matching branch of that submodule's repository. On `main` runs
#      the pin must be on the component's `main` branch strictly; on
#      `staging` runs it may be on `main` or `staging` (an unchanged
#      component keeps its last-released pin).
#
# Usage:  check_submodule_policy.sh <main|staging>
# Exit 0 = policy holds; exit 1 = violations (printed).
#
# Network note: the containment check fetches each submodule's main/staging
# branches. Without network or remotes (e.g. a purely local checkout before
# the organization transfer), the containment check degrades to a warning
# and only the .gitmodules targeting is enforced.
set -uo pipefail

expected="${1:-}"
if [ "$expected" != "main" ] && [ "$expected" != "staging" ]; then
    echo "usage: check_submodule_policy.sh <main|staging>" >&2
    exit 2
fi

submodule_names="kit aero suspension powertrain chassis"
fail=0
containment_skipped=0

for name in $submodule_names; do
    path=$(git config -f .gitmodules --get "submodule.$name.path" 2>/dev/null) || {
        echo "FAIL: submodule '$name' missing from .gitmodules"
        fail=1
        continue
    }

    # --- Check 1: .gitmodules branch tracking -----------------------------
    branch=$(git config -f .gitmodules --get "submodule.$name.branch")
    if [ "$branch" != "$expected" ]; then
        echo "FAIL: .gitmodules [$name] tracks branch '$branch', expected '$expected'"
        fail=1
    fi

    # --- Check 2: pinned commit containment -------------------------------
    pinned=$(git ls-tree HEAD -- "$path" | awk '{print $3}')
    if [ -z "$pinned" ]; then
        echo "FAIL: no pinned commit recorded for '$name' at $path"
        fail=1
        continue
    fi
    if [ ! -e "$path/.git" ]; then
        git submodule update --init "$path" >/dev/null 2>&1 || {
            echo "WARN: cannot init '$name'; skipping containment check"
            containment_skipped=1
            continue
        }
    fi
    if ! git -C "$path" fetch --quiet origin main staging >/dev/null 2>&1 \
            && ! git -C "$path" fetch --quiet origin >/dev/null 2>&1; then
        echo "WARN: cannot fetch '$name' remote branches; skipping containment check"
        containment_skipped=1
        continue
    fi

    if git -C "$path" branch -r --contains "$pinned" 2>/dev/null | grep -q "origin/main"; then
        on_main=yes
    else
        on_main=no
    fi
    if git -C "$path" branch -r --contains "$pinned" 2>/dev/null | grep -q "origin/staging"; then
        on_staging=yes
    else
        on_staging=no
    fi

    if [ "$expected" = "main" ]; then
        if [ "$on_main" != "yes" ]; then
            echo "FAIL: [$name] pinned commit $pinned is not on the component's 'main' branch"
            fail=1
        fi
    else
        if [ "$on_main" != "yes" ] && [ "$on_staging" != "yes" ]; then
            echo "FAIL: [$name] pinned commit $pinned is on neither 'main' nor 'staging' of the component repository"
            fail=1
        fi
    fi
done

if [ "$containment_skipped" -eq 1 ]; then
    echo "NOTE: containment checks were skipped for some submodules (no network/remotes)."
fi
if [ "$fail" -eq 0 ]; then
    echo "OK: submodule policy holds for branch '$expected'."
fi
exit "$fail"
