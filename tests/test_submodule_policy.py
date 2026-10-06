"""Exercise the real policy script with isolated Git failures, without remotes."""

import os
from pathlib import Path
import shutil
import subprocess

import pytest


POLICY = Path(__file__).resolve().parents[1] / "scripts" / "check_submodule_policy.sh"
HARNESS = r'''
export PATH=/usr/bin:/mingw64/bin:$PATH
pin=1111111111111111111111111111111111111111
nested=2222222222222222222222222222222222222222
git() {
    case "$1" in
        config)
            key="${@: -1}"
            if [[ "$key" == *.path ]]; then
                name="${key#submodule.}"
                printf '%s\n' "${name%.path}"
            elif [[ "$MOCK_MODE" == wrong-branch ]]; then
                printf 'feature\n'
            else
                printf '%s\n' "$EXPECTED_BRANCH"
            fi
            ;;
        ls-tree)
            printf '160000 commit %s\t%s\n' "$pin" "${@: -1}"
            ;;
        submodule) return 1 ;;
        -C)
            case "$3" in
                -c)
                    [[ "$MOCK_MODE" != fetch-failure || "$2" != aero ]]
                    ;;
                branch)
                    if [[ "$MOCK_MODE" == invalid-pin && "$2" == aero && "${@: -1}" == "$pin" ]]; then
                        return 0
                    fi
                    [[ "$MOCK_MODE" == staging-only ]] || printf '  origin/main\n'
                    printf '  origin/staging\n'
                    ;;
                ls-tree)
                    [[ "$MOCK_MODE" != missing-nested || "$2" != aero ]] || return 0
                    printf '160000 commit %s\tkit\n' "$nested"
                    ;;
                cat-file) [[ "$MOCK_MODE" != invalid-nested ]] ;;
                *) return 1 ;;
            esac
            ;;
        *) return 1 ;;
    esac
}
source "$1" "$EXPECTED_BRANCH"
'''


@pytest.fixture
def policy_runner(tmp_path):
    if os.name == "nt":
        bash = Path(os.environ.get("ProgramFiles", "C:/Program Files")) / "Git/bin/bash.exe"
        if not bash.is_file():
            pytest.skip("Git Bash is required on Windows")
    else:
        bash = shutil.which("bash")
        if not bash:
            pytest.skip("Bash is required for submodule policy checks")
    harness = tmp_path / "policy_harness.sh"
    harness.write_text(HARNESS, encoding="utf-8", newline="\n")

    def run(mode="ok", branch="staging"):
        for name in ("kit", "aero", "suspension", "powertrain", "chassis"):
            if mode != "init-failure":
                (tmp_path / name / ".git").mkdir(parents=True, exist_ok=True)
        env = dict(os.environ, MOCK_MODE=mode, EXPECTED_BRANCH=branch)
        return subprocess.run(
            [str(bash), "--noprofile", "--norc", str(harness), POLICY.as_posix()],
            cwd=tmp_path, env=env, capture_output=True, text=True, timeout=30,
        )

    return run


@pytest.mark.parametrize("branch", ["main", "staging"])
def test_verified_policy_succeeds(policy_runner, branch):
    result = policy_runner(branch=branch)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "OK: submodule policy holds" in result.stdout


@pytest.mark.parametrize("mode, diagnostic", [
    ("init-failure", "could not be verified"),
    ("fetch-failure", "could not be verified"),
    ("invalid-pin", "is on neither"),
    ("missing-nested", "records no nested kit pin"),
    ("invalid-nested", "nested kit pin"),
    ("wrong-branch", "expected 'staging'"),
])
def test_unverified_or_invalid_policy_fails(policy_runner, mode, diagnostic):
    result = policy_runner(mode)
    assert result.returncode == 1, result.stdout + result.stderr
    assert diagnostic in result.stdout
    assert "OK: submodule policy holds" not in result.stdout


def test_main_rejects_pins_only_on_staging(policy_runner):
    result = policy_runner("staging-only", "main")
    assert result.returncode == 1, result.stdout + result.stderr
    assert "is not on" in result.stdout
