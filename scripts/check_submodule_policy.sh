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
#   3. Each component repository mounts lts-kit at `kit/`; that nested
#      pin must satisfy the same containment rule against lts-kit's
#      branches (strictly `main` on `main` runs, either on `staging` runs).
#
# Usage:  check_submodule_policy.sh <main|staging>
# Exit 0 = policy holds; exit 1 = violations (printed).
#
# Network note: the containment checks fetch each submodule's main/staging
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
kit_fetch_ok=0

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
    # protocol.file.allow=always: no-op for https remotes; required while
    # submodules may resolve from local sibling directories.
    if ! git -C "$path" -c protocol.file.allow=always fetch --quiet origin main staging >/dev/null 2>&1 \
            && ! git -C "$path" -c protocol.file.allow=always fetch --quiet origin >/dev/null 2>&1; then
        echo "WARN: cannot fetch '$name' remote branches; skipping containment check"
        containment_skipped=1
        continue
    fi
    [ "$name" = "kit" ] && kit_fetch_ok=1

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

# --- Check 3: nested kit pins ------------------------------------------
# Each component repository mounts lts-kit at kit/. Containment is
# queried in this checkout's kit working copy, whose origin main/staging
# the loop above fetched; the component working copies only need to be
# at their pinned commits (the loop above ensured that) to read their
# recorded kit gitlink with ls-tree.
kit_path=$(git config -f .gitmodules --get "submodule.kit.path" 2>/dev/null)
if [ -n "$kit_path" ] && [ "$kit_fetch_ok" -eq 1 ] && [ -e "$kit_path/.git" ]; then
    for name in $submodule_names; do
        [ "$name" = "kit" ] && continue
        path=$(git config -f .gitmodules --get "submodule.$name.path" 2>/dev/null)
        nested=$(git -C "$path" ls-tree HEAD -- kit | awk '{print $3}')
        if [ -z "$nested" ]; then
            echo "FAIL: [$name] records no nested kit pin at kit/"
            fail=1
            continue
        fi
        if ! git -C "$kit_path" cat-file -e "$nested^{commit}" 2>/dev/null; then
            echo "FAIL: [$name] nested kit pin $nested is on neither 'main' nor 'staging' of lts-kit"
            fail=1
            continue
        fi
        on_main=no
        on_staging=no
        if git -C "$kit_path" branch -r --contains "$nested" 2>/dev/null | grep -q "origin/main"; then
            on_main=yes
        fi
        if git -C "$kit_path" branch -r --contains "$nested" 2>/dev/null | grep -q "origin/staging"; then
            on_staging=yes
        fi
        if [ "$expected" = "main" ] && [ "$on_main" != "yes" ]; then
            echo "FAIL: [$name] nested kit pin $nested is not on lts-kit's 'main' branch"
            fail=1
        fi
        if [ "$expected" = "staging" ] && [ "$on_main" != "yes" ] && [ "$on_staging" != "yes" ]; then
            echo "FAIL: [$name] nested kit pin $nested is on neither 'main' nor 'staging' of lts-kit"
            fail=1
        fi
    done
else
    echo "NOTE: nested kit pins not checked (kit working copy or its remote branches unavailable)."
    containment_skipped=1
fi

if [ "$containment_skipped" -eq 1 ]; then
    echo "NOTE: containment checks were skipped for some submodules (no network/remotes)."
fi
if [ "$fail" -eq 0 ]; then
    echo "OK: submodule policy holds for branch '$expected'."
fi
exit "$fail"
