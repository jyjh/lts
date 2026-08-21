"""Shared pytest bootstrap for the lts test suite.

Puts the repo's scripts/ directory and the ldparser submodule on sys.path so
test modules can import the extraction scripts and MotecLogGenerator
directly, without each file repeating the setup.
"""

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]

sys.path.insert(0, str(REPO_ROOT / "scripts"))
sys.path.insert(0, str(REPO_ROOT / "external" / "MotecLogGenerator" / "ldparser"))
sys.path.insert(0, str(REPO_ROOT / "external" / "MotecLogGenerator"))
