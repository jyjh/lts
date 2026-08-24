#!/usr/bin/env bash
# release.sh — run a release cascade: merge staging -> main everywhere.
#
# Policy (see the Repositories & Sync documentation page):
#   - Component repositories (lts-kit, lts-aero, lts-suspension,
#     lts-powertrain, lts-chassis) NEVER merge staging -> main on their
#     own. Their `main` branches advance only here, as part of this
#     cascade, when the main repository releases.
#   - The cascade fast-forwards each component's main to its staging tip,
#     then merges the main repository's staging into its main. Because
#     the components' main branches only ever advance through this
#     cascade, the commits pinned on the main repository's staging branch
#     are, after the cascade, commits on the components' main branches —
#     exactly the versions the staging CI proved.
#   - After every merge this script restores the per-branch .gitmodules
#     targeting (main tracks component `main`, staging tracks component
#     `staging`), so a merge can never leave the wrong tracking behind.
#
# Usage:  scripts/release.sh [--dry-run] [--yes]
#   --dry-run  print the plan, change nothing
#   --yes      do not ask for confirmation
#
# Requirements: clean working trees everywhere, and each component's
# `main` must be an ancestor of its `staging` (the script verifies this
# and aborts otherwise). Pushes happen only where an `origin` remote
# exists; run this from the main repository checkout.
set -euo pipefail

DRY_RUN=0
ASSUME_YES=0
for arg in "$@"; do
    case "$arg" in
        --dry-run) DRY_RUN=1 ;;
        --yes) ASSUME_YES=1 ;;
        *) echo "unknown option: $arg" >&2; exit 2 ;;
    esac
done

MAIN_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
COMPONENTS="lts-kit lts-aero lts-suspension lts-powertrain lts-chassis"
SUBMODULES="kit aero suspension powertrain chassis"

component_path() {  # resolve a component repo from .gitmodules relative URL
    local name="$1" url path
    url=$(git -C "$MAIN_ROOT" config -f .gitmodules --get "submodule.$name.url")
    path="$MAIN_ROOT/$url"
    [ -d "$path/.git" ] || { echo "component repo for '$name' not found at $path" >&2; exit 1; }
    echo "$path"
}

run() {  # echo + execute (or just echo in dry-run)
    echo "  \$ $*"
    [ "$DRY_RUN" -eq 1 ] || "$@"
}

commit_if_changed() {  # commit staged/modified tracked files if any remain
    if [ -n "$(git status --porcelain --untracked-files=no)" ]; then
        run git commit -q -m "$1"
    else
        echo "  (nothing to commit)"
    fi
}

set_gitmodules_branch() {  # repo_dir name branch — retarget one submodule entry
    local repo="$1" name="$2" branch="$3"
    [ -f "$repo/.gitmodules" ] || return 0
    run git -C "$repo" config -f .gitmodules "submodule.$name.branch" "$branch"
    run git -C "$repo" add .gitmodules
}

echo "== Release cascade =="
[ "$DRY_RUN" -eq 1 ] && echo "(dry run: nothing will be changed)"

echo
echo "== Preflight =="
for repo in "$MAIN_ROOT" $(for c in $COMPONENTS; do component_path "${c#lts-}"; done); do
    repo_name=$(basename "$repo")
    [ "$DRY_RUN" -eq 1 ] || git -C "$repo" fetch --quiet origin 2>/dev/null || true
    if [ -n "$(git -C "$repo" status --porcelain)" ]; then
        echo "ABORT: '$repo_name' has a dirty working tree." >&2
        exit 1
    fi
    for b in main staging; do
        git -C "$repo" rev-parse --verify -q "$b" >/dev/null || {
            echo "ABORT: '$repo_name' is missing branch '$b'." >&2; exit 1; }
    done
    if ! git -C "$repo" merge-base --is-ancestor main staging; then
        echo "ABORT: '$repo_name' main is NOT an ancestor of staging." \
             "Reconcile first (merge main into staging)." >&2
        exit 1
    fi
    printf '  %-16s main=%s staging=%s\n' "$repo_name" \
        "$(git -C "$repo" rev-parse --short main)" \
        "$(git -C "$repo" rev-parse --short staging)"
done
echo "  preflight OK"

if [ "$ASSUME_YES" -eq 0 ] && [ "$DRY_RUN" -eq 0 ]; then
    read -r -p "Run the release cascade now? [y/N] " answer
    case "$answer" in
        [yY]|[yY][eE][sS]) ;;
        *) echo "aborted."; exit 1 ;;
    esac
fi

echo
echo "== 1/3 Cascade component repositories (fast-forward staging -> main) =="
for c in $COMPONENTS; do
    repo=$(component_path "${c#lts-}")
    echo "-- $c"
    run git -C "$repo" checkout -q main
    run git -C "$repo" merge --ff-only staging
    # After the fast-forward, main's tip carries staging's .gitmodules
    # (kit tracked from staging). Restore main's own targeting.
    set_gitmodules_branch "$repo" kit main
    commit_if_changed "chore(release): main tracks lts-kit main"
    # Bring the normalization commit back into staging, then restore
    # staging's own targeting. Keeps future fast-forwards possible.
    run git -C "$repo" checkout -q staging
    run git -C "$repo" merge --ff-only main
    set_gitmodules_branch "$repo" kit staging
    commit_if_changed "chore(release): staging keeps tracking lts-kit staging"
done

echo
echo "== 2/3 Merge the main repository's staging into main =="
run git -C "$MAIN_ROOT" checkout -q main
if [ "$DRY_RUN" -eq 0 ]; then
    if ! git -C "$MAIN_ROOT" merge --no-ff staging \
            -m "chore(release): merge staging into main"; then
        echo "  merge conflict — resolving to the staging-proven versions"
        for name in $SUBMODULES; do
            path=$(git -C "$MAIN_ROOT" config -f .gitmodules --get "submodule.$name.path")
            git -C "$MAIN_ROOT" checkout --theirs -- "$path" 2>/dev/null || true
        done
        git -C "$MAIN_ROOT" checkout --theirs -- .gitmodules 2>/dev/null || true
        git -C "$MAIN_ROOT" add -A
        git -C "$MAIN_ROOT" commit -q --no-edit
    fi
else
    echo "  \$ git merge --no-ff staging -m 'chore(release): merge staging into main'"
fi
for name in $SUBMODULES; do
    set_gitmodules_branch "$MAIN_ROOT" "$name" main
done
commit_if_changed "chore(release): main tracks component main branches"

echo
echo "== 3/3 Reconcile the main repository's staging =="
run git -C "$MAIN_ROOT" checkout -q staging
if [ "$DRY_RUN" -eq 0 ]; then
    if ! git -C "$MAIN_ROOT" merge --ff-only main; then
        git -C "$MAIN_ROOT" merge main -m "chore(release): bring release into staging" || {
            git -C "$MAIN_ROOT" checkout --ours -- .gitmodules
            git -C "$MAIN_ROOT" add .gitmodules
            git -C "$MAIN_ROOT" commit -q --no-edit
        }
    fi
else
    echo "  \$ git merge --ff-only main (or merge, keeping staging's .gitmodules)"
fi
for name in $SUBMODULES; do
    set_gitmodules_branch "$MAIN_ROOT" "$name" staging
done
commit_if_changed "chore(release): staging keeps tracking component staging branches"
run git -C "$MAIN_ROOT" submodule update --init
run git -C "$MAIN_ROOT" checkout -q main

echo
echo "== Push =="
if [ "$DRY_RUN" -eq 1 ]; then
    echo "  (dry run: no pushes)"
else
    for repo in "$MAIN_ROOT" $(for c in $COMPONENTS; do component_path "${c#lts-}"; done); do
        if git -C "$repo" remote get-url origin >/dev/null 2>&1; then
            echo "  pushing $(basename "$repo")"
            git -C "$repo" push origin main staging
        else
            echo "  $(basename "$repo") has no origin remote — push skipped (pre-transfer local setup)"
        fi
    done
fi

echo
echo "== Done =="
for repo in "$MAIN_ROOT" $(for c in $COMPONENTS; do component_path "${c#lts-}"; done); do
    printf '  %-16s main=%s staging=%s\n' "$(basename "$repo")" \
        "$(git -C "$repo" rev-parse --short main)" \
        "$(git -C "$repo" rev-parse --short staging)"
done
echo
echo "Next: watch CI on every repository's main branch. If any goes red,"
echo "revert that repository's main to the pre-release commit shown in the"
echo "preflight summary above and re-run this script after fixing."
