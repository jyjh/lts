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
# Component repositories: by default the cascade operates on this
# checkout's own submodule working copies (src/+lts/+util, ...), which
# are full clones with origin remotes — `git submodule update --init` is
# the only preparation needed. Alternatively, set LTS_COMPONENTS_ROOT to
# a directory holding sibling clones (lts-kit, lts-aero, ...) and those
# are used instead.
#
# Requirements: clean working trees everywhere; every repository's local
# main/staging branches, where they exist, must match origin; and each
# component's main must be an ancestor of its staging (the script
# verifies all of this and aborts otherwise). Pushes happen only where
# an `origin` remote exists; run this from the main repository checkout.
# main/staging are ruleset-protected; the pushes below succeed only for
# an actor on the rulesets' bypass list (the integration lead).
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
SUBMODULES="kit aero suspension powertrain chassis"

# Authenticate every origin fetch/push as the GitHub CLI user when gh is
# available: the branch rulesets' bypass list names a USER, but git's
# default credential helper (e.g. Git Credential Manager on Windows) may
# hold a different account, and pushes under any other identity are
# rejected. Falls back to git's own credentials when gh is absent.
GIT_AUTH=()
PUSH_IDENTITY="git default credentials (gh not available)"
if command -v gh >/dev/null 2>&1 && gh auth status >/dev/null 2>&1; then
    GIT_AUTH=(-c credential.helper= -c "credential.helper=!gh auth git-credential")
    PUSH_IDENTITY="gh auth: $(gh api user --jq .login 2>/dev/null || echo unknown)"
fi

# Resolve every repository up front and abort before touching anything
# if one is missing — a resolution failure inside the preflight loop
# would otherwise die in a subshell and leave the checks silently
# skipped (that is how "preflight OK" once printed over missing repos).
REPO_DIRS=("$MAIN_ROOT")
REPO_NAMES=(lts)
COMPONENT_DIRS=()
COMPONENT_NAMES=()
for name in $SUBMODULES; do
    if [ -n "${LTS_COMPONENTS_ROOT:-}" ]; then
        dir="$LTS_COMPONENTS_ROOT/lts-$name"
        [ -d "$dir/.git" ] || {
            echo "ABORT: component repo 'lts-$name' not found at $dir." >&2
            exit 1; }
    else
        path=$(git -C "$MAIN_ROOT" config -f .gitmodules --get "submodule.$name.path") || {
            echo "ABORT: submodule '$name' missing from .gitmodules." >&2
            exit 1; }
        dir="$MAIN_ROOT/$path"
        [ -e "$dir/.git" ] || {
            echo "ABORT: no working copy for '$name' at $path." \
                 "Run: git submodule update --init  (or set LTS_COMPONENTS_ROOT)." >&2
            exit 1; }
    fi
    REPO_NAMES+=("lts-$name")
    REPO_DIRS+=("$dir")
    COMPONENT_NAMES+=("lts-$name")
    COMPONENT_DIRS+=("$dir")
done

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
for i in "${!REPO_DIRS[@]}"; do
    repo="${REPO_DIRS[$i]}"
    repo_name="${REPO_NAMES[$i]}"
    has_origin=0
    if git -C "$repo" remote get-url origin >/dev/null 2>&1; then
        has_origin=1
        git ${GIT_AUTH[@]+"${GIT_AUTH[@]}"} -C "$repo" fetch --quiet origin || {
            echo "ABORT: cannot fetch '$repo_name' from origin." >&2
            exit 1; }
    fi
    # --ignore-submodules=dirty: content changes inside submodules (e.g.
    # the phantom CRLF "modified" files the external/ checkouts show on
    # Windows with core.autocrlf=true) do not affect the cascade, which
    # never enters them. Moved submodule pointers (uncommitted bump work)
    # still abort — they appear as ` M` even with this flag.
    if [ -n "$(git -C "$repo" status --porcelain --ignore-submodules=dirty)" ]; then
        echo "ABORT: '$repo_name' has a dirty working tree:" >&2
        git -C "$repo" status --short --ignore-submodules=dirty >&2
        exit 1
    fi
    if [ "$has_origin" -eq 1 ]; then
        for b in main staging; do
            git -C "$repo" rev-parse -q --verify "refs/remotes/origin/$b" >/dev/null || {
                echo "ABORT: '$repo_name' has no origin/$b branch." >&2
                exit 1; }
            # A local branch that exists must be in sync — the cascade
            # both reads and pushes these branches.
            if git -C "$repo" rev-parse -q --verify "refs/heads/$b" >/dev/null; then
                if [ "$(git -C "$repo" rev-parse "refs/heads/$b")" != \
                     "$(git -C "$repo" rev-parse "refs/remotes/origin/$b")" ]; then
                    echo "ABORT: '$repo_name' local '$b' does not match origin/$b — sync first." >&2
                    exit 1
                fi
            fi
        done
        main_ref=refs/remotes/origin/main
        staging_ref=refs/remotes/origin/staging
    else
        for b in main staging; do
            git -C "$repo" rev-parse -q --verify "refs/heads/$b" >/dev/null || {
                echo "ABORT: '$repo_name' is missing branch '$b'." >&2
                exit 1; }
        done
        main_ref=refs/heads/main
        staging_ref=refs/heads/staging
    fi
    if ! git -C "$repo" merge-base --is-ancestor "$main_ref" "$staging_ref"; then
        echo "ABORT: '$repo_name' main is NOT an ancestor of staging." \
             "Reconcile first (merge main into staging)." >&2
        exit 1
    fi
    printf '  %-16s main=%s staging=%s\n' "$repo_name" \
        "$(git -C "$repo" rev-parse --short "$main_ref")" \
        "$(git -C "$repo" rev-parse --short "$staging_ref")"
done
echo "  preflight OK (pushing as: $PUSH_IDENTITY)"

if [ "$ASSUME_YES" -eq 0 ] && [ "$DRY_RUN" -eq 0 ]; then
    read -r -p "Run the release cascade now? [y/N] " answer
    case "$answer" in
        [yY]|[yY][eE][sS]) ;;
        *) echo "aborted."; exit 1 ;;
    esac
fi

echo
echo "== 1/3 Cascade component repositories (fast-forward staging -> main) =="
# Component .gitmodules are branch-agnostic on both branches (no
# branch = lines), so the fast-forward leaves both branches identical —
# no normalization needed.
for i in "${!COMPONENT_DIRS[@]}"; do
    repo="${COMPONENT_DIRS[$i]}"
    repo_name="${COMPONENT_NAMES[$i]}"
    echo "-- $repo_name"
    if git -C "$repo" remote get-url origin >/dev/null 2>&1; then
        run git -C "$repo" checkout -q -B main refs/remotes/origin/main
        run git -C "$repo" merge --ff-only refs/remotes/origin/staging
    else
        run git -C "$repo" checkout -q main
        run git -C "$repo" merge --ff-only staging
    fi
    run git -C "$repo" branch -f staging main
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
            # Resolve only paths that actually conflicted; a bare
            # `checkout --theirs` on a cleanly merged path errors.
            if [ -n "$(git -C "$MAIN_ROOT" ls-files --unmerged -- "$path")" ]; then
                git -C "$MAIN_ROOT" checkout --theirs -- "$path" || {
                    echo "ABORT: could not resolve the submodule pin for '$name'." >&2
                    exit 1; }
                git -C "$MAIN_ROOT" add -- "$path"
            fi
        done
        if [ -n "$(git -C "$MAIN_ROOT" ls-files --unmerged -- .gitmodules)" ]; then
            git -C "$MAIN_ROOT" checkout --theirs -- .gitmodules || {
                echo "ABORT: could not resolve .gitmodules." >&2
                exit 1; }
        fi
        if [ -n "$(git -C "$MAIN_ROOT" ls-files --unmerged)" ]; then
            echo "ABORT: conflicts remain outside .gitmodules and the submodule pins." >&2
            echo "  Nothing has been pushed, but step 1 already fast-forwarded the" >&2
            echo "  component repositories' local main branches. Resolve the merge" >&2
            echo "  by hand, push it, and re-run this script (step 1 is idempotent)." >&2
            exit 1
        fi
        # Stage only the resolved files: a bare `git add -A` here could
        # sweep in submodule working-copy states left behind by step 1
        # and record pin bumps that were never tested.
        git -C "$MAIN_ROOT" add .gitmodules
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
            if [ -n "$(git -C "$MAIN_ROOT" ls-files --unmerged -- .gitmodules)" ]; then
                git -C "$MAIN_ROOT" checkout --ours -- .gitmodules
                git -C "$MAIN_ROOT" add .gitmodules
            fi
            if [ -n "$(git -C "$MAIN_ROOT" ls-files --unmerged)" ]; then
                echo "ABORT: unresolved conflicts bringing the release into staging." >&2
                echo "  Nothing has been pushed, but step 1 already fast-forwarded the" >&2
                echo "  component repositories' local main branches. Resolve by hand," >&2
                echo "  push, and re-run this script (step 1 is idempotent)." >&2
                exit 1
            fi
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
# protocol.file.allow=always: no-op for https remotes; required while
# submodules may resolve from local sibling directories.
run git -C "$MAIN_ROOT" -c protocol.file.allow=always submodule update --init
run git -C "$MAIN_ROOT" checkout -q main

echo
echo "== Push =="
if [ "$DRY_RUN" -eq 1 ]; then
    echo "  (dry run: no pushes)"
else
    # Push the main repository last: its submodule pins reference the
    # components, so their new main branches should exist first. (The
    # pinned commits are already on the components' remotes from their
    # staging pushes — this is ordering hygiene, not correctness.)
    for ((i=${#REPO_DIRS[@]} - 1; i >= 0; i--)); do
        repo="${REPO_DIRS[$i]}"
        repo_name="${REPO_NAMES[$i]}"
        if git -C "$repo" remote get-url origin >/dev/null 2>&1; then
            echo "  pushing $repo_name"
            git ${GIT_AUTH[@]+"${GIT_AUTH[@]}"} -C "$repo" push origin main staging
        else
            echo "  $repo_name has no origin remote — push skipped (pre-transfer local setup)"
        fi
    done
fi

echo
echo "== Done =="
for i in "${!REPO_DIRS[@]}"; do
    printf '  %-16s main=%s staging=%s\n' "${REPO_NAMES[$i]}" \
        "$(git -C "${REPO_DIRS[$i]}" rev-parse --short main)" \
        "$(git -C "${REPO_DIRS[$i]}" rev-parse --short staging)"
done
echo
echo "Next: watch CI on every repository's main branch. If any goes red,"
echo "revert that repository's main to the pre-release commit shown in the"
echo "preflight summary above and re-run this script after fixing."
