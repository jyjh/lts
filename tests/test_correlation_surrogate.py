import csv
import json
import sys
import uuid
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import correlation_surrogate as surrogate  # noqa: E402


SPACE = ROOT / "config" / "correlation" / "lap5_ml_parameter_space.json"


def test_initial_design_is_deterministic_and_starts_at_baseline():
    space = surrogate.load_space(SPACE)
    first = surrogate.initial_candidates(space, 12, 25)
    second = surrogate.initial_candidates(space, 12, 25)
    assert np.array_equal(first, second)
    assert np.allclose(first[0], surrogate.baseline(space))
    normalized = surrogate.encode(space, first)
    assert np.all((normalized >= 0) & (normalized <= 1))


def temporary_test_file(suffix: str) -> Path:
    return ROOT / "tests" / f".correlation_surrogate_{uuid.uuid4().hex}{suffix}"


def test_surrogate_proposes_bounded_diverse_candidates():
    space = surrogate.load_space(SPACE)
    values = surrogate.initial_candidates(space, 40, 9)
    normalized = surrogate.encode(space, values)
    scores = np.sum((normalized - 0.35) ** 2, axis=1)
    history = temporary_test_file(".csv")
    names = [param["name"] for param in space["parameters"]]
    with history.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["candidate_id", *names, "score"])
        for index, (row, score) in enumerate(zip(values, scores), 1):
            writer.writerow([index, *row, score])

    try:
        proposed, metadata = surrogate.propose_candidates(
            space, history, 6, seed=10, pool_size=2048
        )
        encoded = surrogate.encode(space, proposed)
        assert proposed.shape == (6, len(names))
        assert np.all((encoded >= 0) & (encoded <= 1))
        assert len(np.unique(encoded, axis=0)) == 6
        assert metadata["trainingRows"] == 40
        assert set(metadata["featureImportances"]) == set(names)
    finally:
        history.unlink(missing_ok=True)


def test_cli_writes_candidate_csv():
    output = temporary_test_file(".csv")
    try:
        result = surrogate.main(
            [
                "initial",
                "--space",
                str(SPACE),
                "--count",
                "4",
                "--seed",
                "3",
                "--output",
                str(output),
            ]
        )
        assert result == 0
        rows = list(csv.DictReader(output.open(encoding="utf-8")))
        assert len(rows) == 4
        assert rows[0]["candidate_id"] == "1"
    finally:
        output.unlink(missing_ok=True)
