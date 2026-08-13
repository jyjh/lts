#!/usr/bin/env python3
"""Generate correlation candidates with a deterministic Extra Trees surrogate."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
from scipy.stats import qmc
from sklearn.ensemble import ExtraTreesRegressor


SCHEMA = "lts.correlation.parameter-space.v1"


def load_space(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("schema") != SCHEMA:
        raise ValueError(f"Expected parameter-space schema {SCHEMA}")
    params = data.get("parameters", [])
    if not params:
        raise ValueError("Parameter space contains no parameters")
    return data


def decode(space: dict[str, Any], normalized: np.ndarray) -> np.ndarray:
    normalized = np.asarray(normalized, dtype=float)
    physical = np.empty_like(normalized)
    for index, param in enumerate(space["parameters"]):
        lower = float(param["lower"])
        upper = float(param["upper"])
        if param["transform"].lower() == "log":
            physical[:, index] = np.exp(
                np.log(lower) + normalized[:, index] * (np.log(upper) - np.log(lower))
            )
        else:
            physical[:, index] = lower + normalized[:, index] * (upper - lower)
    return physical


def encode(space: dict[str, Any], physical: np.ndarray) -> np.ndarray:
    physical = np.asarray(physical, dtype=float)
    normalized = np.empty_like(physical)
    for index, param in enumerate(space["parameters"]):
        lower = float(param["lower"])
        upper = float(param["upper"])
        values = physical[:, index]
        if param["transform"].lower() == "log":
            normalized[:, index] = (np.log(values) - np.log(lower)) / (
                np.log(upper) - np.log(lower)
            )
        else:
            normalized[:, index] = (values - lower) / (upper - lower)
    return normalized


def baseline(space: dict[str, Any]) -> np.ndarray:
    return np.asarray([float(param["baseline"]) for param in space["parameters"]])


def initial_candidates(space: dict[str, Any], count: int, seed: int) -> np.ndarray:
    if count < 1:
        raise ValueError("count must be positive")
    dimension = len(space["parameters"])
    if count == 1:
        return baseline(space)[None, :]
    sampler = qmc.LatinHypercube(d=dimension, seed=seed)
    design = sampler.random(count - 1)
    return np.vstack([baseline(space), decode(space, design)])


def read_history(
    path: Path, space: dict[str, Any]
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    names = [param["name"] for param in space["parameters"]]
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        rows = list(csv.DictReader(handle))
    finite_rows = []
    for row in rows:
        try:
            score = float(row["score"])
            values = [float(row[name]) for name in names]
        except (KeyError, TypeError, ValueError):
            continue
        if math.isfinite(score) and np.all(np.isfinite(values)):
            finite_rows.append((int(float(row["candidate_id"])), score, values))
    if len(finite_rows) < 2:
        raise ValueError("At least two finite history rows are required")
    ids = np.asarray([row[0] for row in finite_rows], dtype=int)
    scores = np.asarray([row[1] for row in finite_rows], dtype=float)
    physical = np.asarray([row[2] for row in finite_rows], dtype=float)
    return ids, scores, physical


def _tree_predictions(model: ExtraTreesRegressor, values: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    mean = np.zeros(values.shape[0])
    second_moment = np.zeros(values.shape[0])
    for estimator in model.estimators_:
        prediction = estimator.predict(values)
        mean += prediction
        second_moment += prediction * prediction
    mean /= len(model.estimators_)
    variance = np.maximum(0.0, second_moment / len(model.estimators_) - mean * mean)
    return mean, np.sqrt(variance)


def propose_candidates(
    space: dict[str, Any],
    history_path: Path,
    count: int,
    seed: int,
    pool_size: int = 100_000,
) -> tuple[np.ndarray, dict[str, Any]]:
    ids, scores, physical = read_history(history_path, space)
    x = encode(space, physical)
    model = ExtraTreesRegressor(
        n_estimators=384,
        min_samples_leaf=2,
        max_features=0.8,
        bootstrap=False,
        # MATLAB parallelizes expensive simulator evaluations. Keeping the
        # lightweight surrogate fit in-process avoids Windows/joblib named
        # pipe restrictions and makes CLI behavior reproducible.
        n_jobs=1,
        random_state=seed,
    )
    model.fit(x, scores)

    sampler = qmc.Sobol(d=x.shape[1], scramble=True, seed=seed)
    exponent = int(math.ceil(math.log2(max(pool_size, count))))
    pool = sampler.random_base2(exponent)[:pool_size]
    mean, uncertainty = _tree_predictions(model, pool)
    acquisition = mean - 0.75 * uncertainty
    order = np.argsort(acquisition)

    selected: list[np.ndarray] = []
    minimum_distance = max(0.015, 0.08 / math.sqrt(x.shape[1]))
    for index in order:
        candidate = pool[index]
        existing_distance = np.min(np.linalg.norm(x - candidate, axis=1))
        selected_distance = (
            min(np.linalg.norm(chosen - candidate) for chosen in selected)
            if selected
            else math.inf
        )
        if existing_distance >= minimum_distance and selected_distance >= minimum_distance:
            selected.append(candidate)
        if len(selected) == count:
            break
    if len(selected) < count:
        used = {tuple(row) for row in selected}
        for index in order:
            key = tuple(pool[index])
            if key not in used:
                selected.append(pool[index])
                used.add(key)
            if len(selected) == count:
                break

    metadata = {
        "trainingRows": int(len(scores)),
        "bestObservedScore": float(np.min(scores)),
        "minimumDiversityDistance": minimum_distance,
        "acquisition": "mean - 0.75 * ensemble_std",
        "featureImportances": {
            param["name"]: float(value)
            for param, value in zip(space["parameters"], model.feature_importances_)
        },
        "nextCandidateId": int(np.max(ids) + 1),
    }
    return decode(space, np.asarray(selected)), metadata


def write_candidates(
    path: Path, space: dict[str, Any], values: np.ndarray, first_id: int
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    names = [param["name"] for param in space["parameters"]]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["candidate_id", *names])
        for offset, row in enumerate(values):
            writer.writerow([first_id + offset, *[f"{value:.17g}" for value in row]])


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    for name in ("initial", "propose"):
        command = subparsers.add_parser(name)
        command.add_argument("--space", type=Path, required=True)
        command.add_argument("--count", type=int, required=True)
        command.add_argument("--seed", type=int, required=True)
        command.add_argument("--output", type=Path, required=True)
        if name == "propose":
            command.add_argument("--history", type=Path, required=True)
            command.add_argument("--metadata", type=Path)
            command.add_argument("--pool-size", type=int, default=100_000)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    space = load_space(args.space)
    if args.command == "initial":
        values = initial_candidates(space, args.count, args.seed)
        first_id = 1
    else:
        values, metadata = propose_candidates(
            space, args.history, args.count, args.seed, args.pool_size
        )
        first_id = int(metadata["nextCandidateId"])
        if args.metadata:
            args.metadata.parent.mkdir(parents=True, exist_ok=True)
            args.metadata.write_text(
                json.dumps(metadata, indent=2, sort_keys=True), encoding="utf-8"
            )
    write_candidates(args.output, space, values, first_id)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
