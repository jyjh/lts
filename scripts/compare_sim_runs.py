#!/usr/bin/env python3
"""Compare two simulator telemetry CSV files over the same track.

The helper is intentionally dependency-free for normal MoTeC CSV exports. It
can optionally use SciPy to read MATLAB .mat track files, but the exported CSVs
already contain curvature channels, so a track file is usually only metadata.
"""

from __future__ import annotations

import argparse
import csv
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


G_MPS2 = 9.80665


@dataclass
class HeaderInfo:
    name: str
    base: str
    unit: str
    normalized_base: str
    normalized_full: str


@dataclass
class TelemetryRun:
    path: Path
    label: str
    distance_m: List[float]
    time_s: Optional[List[float]]
    speed_kmh: List[float]
    ax_mps2: Optional[List[float]]
    ay_mps2: Optional[List[float]]
    curvature_1pm: Optional[List[float]]
    throttle_pct: Optional[List[float]]
    brake_pct: Optional[List[float]]


@dataclass
class TrackCurvature:
    label: str
    station_m: List[float]
    curvature_1pm: List[float]
    warning: Optional[str] = None


@dataclass
class ReportRow:
    area: str
    stronger: str
    a_value: str
    b_value: str
    delta: str
    basis: str


def normalize_name(value: str) -> str:
    return re.sub(r"[^a-z0-9]+", "", value.lower())


def split_header(value: str) -> Tuple[str, str]:
    value = value.strip()
    match = re.match(r"^(.*?)\s*\(([^()]*)\)\s*$", value)
    if not match:
        return value, ""
    return match.group(1).strip(), match.group(2).strip()


def parse_headers(headers: Sequence[str]) -> List[HeaderInfo]:
    parsed: List[HeaderInfo] = []
    for name in headers:
        base, unit = split_header(name)
        parsed.append(
            HeaderInfo(
                name=name,
                base=base,
                unit=unit,
                normalized_base=normalize_name(base),
                normalized_full=normalize_name(name),
            )
        )
    return parsed


def find_column(headers: Sequence[HeaderInfo], aliases: Iterable[str]) -> Optional[int]:
    normalized_aliases = [normalize_name(alias) for alias in aliases]
    for alias in normalized_aliases:
        for idx, header in enumerate(headers):
            if header.normalized_base == alias or header.normalized_full == alias:
                return idx
    return None


def parse_float(value: str) -> float:
    value = value.strip()
    if not value:
        return math.nan
    try:
        return float(value)
    except ValueError:
        return math.nan


def unit_has(unit: str, token: str) -> bool:
    return normalize_name(token) in normalize_name(unit)


def speed_scale(header: HeaderInfo) -> float:
    unit = header.unit
    if unit_has(unit, "km/h") or unit_has(unit, "kmh"):
        return 1.0
    if unit_has(unit, "mph"):
        return 1.609344
    if unit_has(unit, "m/s") or unit_has(unit, "mps"):
        return 3.6
    if normalize_name(header.base) in {"speedmps", "vx", "vy"}:
        return 3.6
    return 1.0


def accel_scale(header: HeaderInfo) -> float:
    unit = normalize_name(header.unit)
    if unit == "g":
        return G_MPS2
    return 1.0


def percent_scale(header: HeaderInfo) -> float:
    unit = normalize_name(header.unit)
    if unit in {"ratio", "fraction"}:
        return 100.0
    return 1.0


def finite(value: float) -> bool:
    return math.isfinite(value)


def read_optional(
    row: Sequence[str],
    idx: Optional[int],
    scale: float,
) -> float:
    if idx is None or idx >= len(row):
        return math.nan
    return parse_float(row[idx]) * scale


def all_nan(values: List[float]) -> bool:
    return not any(finite(value) for value in values)


def clean_by_distance(
    distance: List[float],
    channels: Dict[str, Optional[List[float]]],
) -> Tuple[List[float], Dict[str, Optional[List[float]]]]:
    cleaned_distance: List[float] = []
    cleaned_channels: Dict[str, Optional[List[float]]] = {
        name: ([] if values is not None else None) for name, values in channels.items()
    }
    last_distance = -math.inf

    for idx, station in enumerate(distance):
        if not finite(station) or station <= last_distance + 1e-9:
            continue

        speed_values = channels.get("speed_kmh")
        if speed_values is None or idx >= len(speed_values) or not finite(speed_values[idx]):
            continue

        cleaned_distance.append(station)
        last_distance = station
        for name, values in channels.items():
            if values is not None:
                cleaned_channels[name].append(values[idx])

    return cleaned_distance, cleaned_channels


def derive_ax_from_speed(
    speed_kmh: List[float],
    time_s: Optional[List[float]],
) -> Optional[List[float]]:
    if time_s is None or len(time_s) != len(speed_kmh):
        return None

    speed_mps = [speed / 3.6 for speed in speed_kmh]
    ax = [0.0] * len(speed_mps)
    for idx in range(len(speed_mps)):
        left = max(0, idx - 1)
        right = min(len(speed_mps) - 1, idx + 1)
        dt = time_s[right] - time_s[left]
        if dt > 0:
            ax[idx] = (speed_mps[right] - speed_mps[left]) / dt
        else:
            ax[idx] = math.nan
    return ax


def normalize_percent_channel(values: Optional[List[float]]) -> Optional[List[float]]:
    if values is None or all_nan(values):
        return None

    finite_values = [value for value in values if finite(value)]
    if finite_values and max(finite_values) <= 1.5:
        return [value * 100.0 if finite(value) else value for value in values]
    return values


def infer_label(path: Path) -> str:
    stem = path.stem
    match = re.match(r"^motec_([^_]+)_(.+?)_\d{8}_\d{6}$", stem)
    if match:
        return match.group(2)
    return stem


def read_telemetry_csv(path: Path, label: Optional[str] = None) -> TelemetryRun:
    if not path.exists():
        raise FileNotFoundError(f'Telemetry CSV "{path}" does not exist.')

    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.reader(handle)
        try:
            raw_headers = next(reader)
        except StopIteration as exc:
            raise ValueError(f'Telemetry CSV "{path}" is empty.') from exc

        headers = parse_headers(raw_headers)

        distance_idx = find_column(headers, ["Distance", "s", "Ref S", "Control Distance"])
        time_idx = find_column(headers, ["Time", "Control Time"])
        speed_idx = find_column(headers, [
            "Simulation Vehicle Speed Value",
            "Vehicle Speed Value",
            "Speed Kmh",
            "speedKmh",
        ])
        speed_mps_idx = find_column(headers, ["Speed mps", "Speed", "VX"])
        ax_idx = find_column(headers, ["Long Accel Raw", "ax", "Target Long Accel"])
        ax_g_idx = find_column(headers, ["G Sensor Front Acceleration Longitudinal"])
        ay_idx = find_column(headers, ["Lat Accel Raw", "ay"])
        ay_g_idx = find_column(headers, ["G Sensor Front Acceleration Lateral"])
        curvature_idx = find_column(headers, ["Ref Curvature", "Racing Line Curvature", "Curvature", "lineCurvature"])
        throttle_idx = find_column(headers, ["Throttle Pedal", "Throttle Raw", "throttle"])
        brake_idx = find_column(headers, ["Brake", "Brake Requested", "Brake Raw", "brake", "brakeRequested"])

        if distance_idx is None:
            raise ValueError(f'Telemetry CSV "{path}" has no distance column.')
        if speed_idx is None and speed_mps_idx is None:
            raise ValueError(f'Telemetry CSV "{path}" has no speed column.')

        active_speed_idx = speed_idx if speed_idx is not None else speed_mps_idx
        assert active_speed_idx is not None

        active_ax_idx = ax_idx if ax_idx is not None else ax_g_idx
        active_ay_idx = ay_idx if ay_idx is not None else ay_g_idx

        speed_multiplier = speed_scale(headers[active_speed_idx])
        ax_multiplier = accel_scale(headers[active_ax_idx]) if active_ax_idx is not None else 1.0
        ay_multiplier = accel_scale(headers[active_ay_idx]) if active_ay_idx is not None else 1.0
        throttle_multiplier = percent_scale(headers[throttle_idx]) if throttle_idx is not None else 1.0
        brake_multiplier = percent_scale(headers[brake_idx]) if brake_idx is not None else 1.0

        distance: List[float] = []
        time_s: List[float] = []
        speed_kmh: List[float] = []
        ax_mps2: List[float] = []
        ay_mps2: List[float] = []
        curvature_1pm: List[float] = []
        throttle_pct: List[float] = []
        brake_pct: List[float] = []

        for row in reader:
            distance.append(read_optional(row, distance_idx, 1.0))
            time_s.append(read_optional(row, time_idx, 1.0))
            speed_kmh.append(read_optional(row, active_speed_idx, speed_multiplier))
            ax_mps2.append(read_optional(row, active_ax_idx, ax_multiplier))
            ay_mps2.append(read_optional(row, active_ay_idx, ay_multiplier))
            curvature_1pm.append(read_optional(row, curvature_idx, 1.0))
            throttle_pct.append(read_optional(row, throttle_idx, throttle_multiplier))
            brake_pct.append(read_optional(row, brake_idx, brake_multiplier))

    channels: Dict[str, Optional[List[float]]] = {
        "time_s": None if all_nan(time_s) else time_s,
        "speed_kmh": speed_kmh,
        "ax_mps2": None if all_nan(ax_mps2) else ax_mps2,
        "ay_mps2": None if all_nan(ay_mps2) else ay_mps2,
        "curvature_1pm": None if all_nan(curvature_1pm) else curvature_1pm,
        "throttle_pct": None if all_nan(throttle_pct) else throttle_pct,
        "brake_pct": None if all_nan(brake_pct) else brake_pct,
    }

    clean_distance, clean_channels = clean_by_distance(distance, channels)
    if len(clean_distance) < 2:
        raise ValueError(f'Telemetry CSV "{path}" has fewer than two usable distance samples.')

    ax_clean = clean_channels["ax_mps2"]
    if ax_clean is None:
        ax_clean = derive_ax_from_speed(clean_channels["speed_kmh"], clean_channels["time_s"])

    return TelemetryRun(
        path=path,
        label=label or infer_label(path),
        distance_m=clean_distance,
        time_s=clean_channels["time_s"],
        speed_kmh=clean_channels["speed_kmh"],
        ax_mps2=ax_clean,
        ay_mps2=clean_channels["ay_mps2"],
        curvature_1pm=clean_channels["curvature_1pm"],
        throttle_pct=normalize_percent_channel(clean_channels["throttle_pct"]),
        brake_pct=normalize_percent_channel(clean_channels["brake_pct"]),
    )


def cumulative_arc_length(points: Sequence[Tuple[float, float]], closed: bool) -> List[float]:
    stations = [0.0]
    last = points[0]
    for point in points[1:]:
        stations.append(stations[-1] + math.hypot(point[0] - last[0], point[1] - last[1]))
        last = point
    if closed and len(points) > 2:
        stations.append(stations[-1] + math.hypot(points[0][0] - last[0], points[0][1] - last[1]))
    return stations


def compute_curvature(points: Sequence[Tuple[float, float]], closed: bool) -> List[float]:
    n_points = len(points)
    curvature = [0.0] * n_points
    if n_points < 3:
        return curvature

    for idx in range(n_points):
        if closed:
            prev_idx = (idx - 1) % n_points
            next_idx = (idx + 1) % n_points
        else:
            prev_idx = max(0, idx - 1)
            next_idx = min(n_points - 1, idx + 1)
            if prev_idx == idx or next_idx == idx:
                continue

        px, py = points[prev_idx]
        cx, cy = points[idx]
        nx, ny = points[next_idx]
        avec = (cx - px, cy - py)
        bvec = (nx - cx, ny - cy)
        cvec = (nx - px, ny - py)
        a = math.hypot(*avec)
        b = math.hypot(*bvec)
        c = math.hypot(*cvec)
        denom = a * b * c
        if denom > 0:
            area2 = avec[0] * bvec[1] - avec[1] * bvec[0]
            curvature[idx] = 2.0 * area2 / denom

    if not closed and n_points >= 3:
        curvature[0] = curvature[1]
        curvature[-1] = curvature[-2]
    return curvature


def list_from_mat_value(value: object) -> List[float]:
    try:
        flattened = value.ravel()
    except AttributeError:
        flattened = value
    return [float(item) for item in flattened]


def load_track_csv(path: Path) -> TrackCurvature:
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.reader(handle)
        try:
            raw_headers = next(reader)
        except StopIteration as exc:
            raise ValueError(f'Track CSV "{path}" is empty.') from exc

        headers = parse_headers(raw_headers)
        station_idx = find_column(headers, ["station_m", "s_m", "Distance", "s"])
        curvature_idx = find_column(headers, ["curvature_1pm", "Curvature", "Ref Curvature"])
        if station_idx is None or curvature_idx is None:
            raise ValueError(
                f'Track CSV "{path}" must contain station/distance and curvature columns.'
            )

        station_m: List[float] = []
        curvature_1pm: List[float] = []
        for row in reader:
            station = read_optional(row, station_idx, 1.0)
            curvature = read_optional(row, curvature_idx, 1.0)
            if finite(station) and finite(curvature):
                station_m.append(station)
                curvature_1pm.append(curvature)

    if len(station_m) < 2:
        raise ValueError(f'Track CSV "{path}" has fewer than two usable curvature samples.')
    return TrackCurvature(str(path), station_m, curvature_1pm)


def load_track_mat(path: Path) -> TrackCurvature:
    try:
        from scipy.io import loadmat  # type: ignore
    except ImportError as exc:
        raise RuntimeError(
            "SciPy is required to read .mat track files from Python. "
            "Install scipy, export the track to CSV, or rely on telemetry curvature."
        ) from exc

    data = loadmat(str(path), squeeze_me=True, struct_as_record=False)
    if "track" not in data:
        raise ValueError(f'MAT file "{path}" does not contain a variable named "track".')

    track = data["track"]
    station_m: Optional[List[float]] = None
    curvature_1pm: Optional[List[float]] = None

    if hasattr(track, "station_m"):
        station_m = list_from_mat_value(track.station_m)
    if hasattr(track, "curvature_1pm"):
        curvature_1pm = list_from_mat_value(track.curvature_1pm)

    if curvature_1pm is None or station_m is None:
        if not hasattr(track, "points_m"):
            raise ValueError(
                f'MAT file "{path}" must contain track.station_m/track.curvature_1pm '
                "or track.points_m."
            )
        raw_points = track.points_m
        points = [(float(row[0]), float(row[1])) for row in raw_points]
        closed = bool(getattr(track, "closed", True))
        if station_m is None:
            station_m = cumulative_arc_length(points, closed)[: len(points)]
        if curvature_1pm is None:
            curvature_1pm = compute_curvature(points, closed)

    if len(station_m) != len(curvature_1pm):
        n_samples = min(len(station_m), len(curvature_1pm))
        station_m = station_m[:n_samples]
        curvature_1pm = curvature_1pm[:n_samples]

    if len(station_m) < 2:
        raise ValueError(f'MAT file "{path}" has fewer than two usable curvature samples.')
    return TrackCurvature(str(path), station_m, curvature_1pm)


def load_track_curvature(track_arg: Optional[str]) -> Optional[TrackCurvature]:
    if not track_arg:
        return None

    path = Path(track_arg)
    if not path.exists():
        return TrackCurvature(
            label=track_arg,
            station_m=[],
            curvature_1pm=[],
            warning=f'Track "{track_arg}" is not a readable file; using telemetry curvature if available.',
        )

    try:
        if path.suffix.lower() == ".csv":
            return load_track_csv(path)
        if path.suffix.lower() == ".mat":
            return load_track_mat(path)
        return TrackCurvature(
            label=str(path),
            station_m=[],
            curvature_1pm=[],
            warning=f'Track file "{path}" is not .csv or .mat; using telemetry curvature if available.',
        )
    except Exception as exc:
        return TrackCurvature(
            label=str(path),
            station_m=[],
            curvature_1pm=[],
            warning=f'Could not load track curvature from "{path}": {exc}',
        )


def interpolate_series(x_values: List[float], y_values: List[float], grid: List[float]) -> List[float]:
    result: List[float] = []
    idx = 0
    last_idx = len(x_values) - 1

    for x in grid:
        while idx < last_idx - 1 and x_values[idx + 1] < x:
            idx += 1

        x0 = x_values[idx]
        x1 = x_values[min(idx + 1, last_idx)]
        y0 = y_values[idx]
        y1 = y_values[min(idx + 1, last_idx)]
        if x1 <= x0 or not finite(y0) or not finite(y1):
            result.append(y0)
            continue
        ratio = (x - x0) / (x1 - x0)
        result.append(y0 + ratio * (y1 - y0))
    return result


def build_grid(start: float, end: float, step: float) -> List[float]:
    if step <= 0:
        raise ValueError("--distance-step must be positive.")
    n_steps = int(math.floor((end - start) / step))
    grid = [start + idx * step for idx in range(n_steps + 1)]
    if not grid or grid[-1] < end - 1e-9:
        grid.append(end)
    return grid


def mean(values: Iterable[float]) -> Optional[float]:
    count = 0
    total = 0.0
    for value in values:
        if finite(value):
            total += value
            count += 1
    if count == 0:
        return None
    return total / count


def max_finite(values: Iterable[float]) -> Optional[float]:
    best: Optional[float] = None
    for value in values:
        if finite(value) and (best is None or value > best):
            best = value
    return best


def value_text(value: Optional[float], unit: str, precision: int = 2) -> str:
    if value is None or not finite(value):
        return "n/a"
    return f"{value:.{precision}f} {unit}".strip()


def winner_text(
    a_label: str,
    b_label: str,
    a_value: Optional[float],
    b_value: Optional[float],
    higher_is_better: bool,
    tolerance: float,
) -> str:
    if a_value is None or b_value is None:
        return "n/a"
    delta = b_value - a_value
    if abs(delta) <= tolerance:
        return "Tie"
    if higher_is_better:
        return b_label if delta > 0 else a_label
    return b_label if delta < 0 else a_label


def delta_text(
    a_value: Optional[float],
    b_value: Optional[float],
    unit: str,
    precision: int = 2,
) -> str:
    if a_value is None or b_value is None:
        return "n/a"
    delta = b_value - a_value
    return f"{delta:+.{precision}f} {unit}".strip()


def make_row(
    area: str,
    a_label: str,
    b_label: str,
    a_value: Optional[float],
    b_value: Optional[float],
    unit: str,
    basis: str,
    higher_is_better: bool = True,
    tolerance: float = 0.0,
    precision: int = 2,
) -> ReportRow:
    return ReportRow(
        area=area,
        stronger=winner_text(a_label, b_label, a_value, b_value, higher_is_better, tolerance),
        a_value=value_text(a_value, unit, precision),
        b_value=value_text(b_value, unit, precision),
        delta=delta_text(a_value, b_value, unit, precision),
        basis=basis,
    )


def mask_mean(values: List[float], mask: List[bool]) -> Optional[float]:
    return mean(value for value, keep in zip(values, mask) if keep)


def mask_distance(grid: List[float], mask: List[bool]) -> float:
    if len(grid) < 2 or len(mask) < 2:
        return 0.0

    total = 0.0
    for idx in range(len(grid) - 1):
        if mask[idx] or mask[idx + 1]:
            total += grid[idx + 1] - grid[idx]
    return total


def combine_optional_mean(a: Optional[List[float]], b: Optional[List[float]], default: float) -> List[float]:
    if a is None and b is None:
        return []
    length = len(a if a is not None else b)
    combined: List[float] = []
    for idx in range(length):
        values = []
        if a is not None and finite(a[idx]):
            values.append(a[idx])
        if b is not None and finite(b[idx]):
            values.append(b[idx])
        combined.append(sum(values) / len(values) if values else default)
    return combined


def format_distance(distance_m: float) -> str:
    if distance_m >= 1000:
        return f"{distance_m / 1000:.2f} km"
    return f"{distance_m:.1f} m"


def compare_runs(
    run_a: TelemetryRun,
    run_b: TelemetryRun,
    track: Optional[TrackCurvature],
    distance_step: float,
    corner_curvature_threshold: float,
    throttle_threshold: float,
    brake_threshold: float,
) -> str:
    start = max(run_a.distance_m[0], run_b.distance_m[0])
    end = min(run_a.distance_m[-1], run_b.distance_m[-1])
    if end <= start:
        raise ValueError("The two runs do not have overlapping distance ranges.")

    grid = build_grid(start, end, distance_step)
    speed_a = interpolate_series(run_a.distance_m, run_a.speed_kmh, grid)
    speed_b = interpolate_series(run_b.distance_m, run_b.speed_kmh, grid)
    time_a = interpolate_series(run_a.distance_m, run_a.time_s, grid) if run_a.time_s else None
    time_b = interpolate_series(run_b.distance_m, run_b.time_s, grid) if run_b.time_s else None
    ax_a = interpolate_series(run_a.distance_m, run_a.ax_mps2, grid) if run_a.ax_mps2 else None
    ax_b = interpolate_series(run_b.distance_m, run_b.ax_mps2, grid) if run_b.ax_mps2 else None
    throttle_a = (
        interpolate_series(run_a.distance_m, run_a.throttle_pct, grid) if run_a.throttle_pct else None
    )
    throttle_b = (
        interpolate_series(run_b.distance_m, run_b.throttle_pct, grid) if run_b.throttle_pct else None
    )
    brake_a = interpolate_series(run_a.distance_m, run_a.brake_pct, grid) if run_a.brake_pct else None
    brake_b = interpolate_series(run_b.distance_m, run_b.brake_pct, grid) if run_b.brake_pct else None

    curvature_source = "none"
    curvature_warning: Optional[str] = None
    if track is not None and track.warning:
        curvature_warning = track.warning

    if track is not None and track.station_m and track.curvature_1pm:
        curvature = interpolate_series(track.station_m, track.curvature_1pm, grid)
        curvature_source = f"track file ({track.label})"
    else:
        curvature_a = (
            interpolate_series(run_a.distance_m, run_a.curvature_1pm, grid)
            if run_a.curvature_1pm
            else None
        )
        curvature_b = (
            interpolate_series(run_b.distance_m, run_b.curvature_1pm, grid)
            if run_b.curvature_1pm
            else None
        )
        if curvature_a is None and curvature_b is None:
            raise ValueError(
                "No curvature source is available. Include Ref Curvature/Curvature in the "
                "telemetry CSVs or pass a track CSV/MAT with curvature."
            )

        curvature = combine_optional_mean(curvature_a, curvature_b, 0.0)
        curvature_source = "telemetry curvature"

    abs_curvature = [abs(value) if finite(value) else 0.0 for value in curvature]
    avg_speed = [(a + b) / 2.0 for a, b in zip(speed_a, speed_b)]
    corner_mask = [value >= corner_curvature_threshold for value in abs_curvature]

    rows: List[ReportRow] = []
    corner_bins = [
        ("Low speed corners (0-30 km/h)", 0.0, 30.0),
        ("Medium speed corners (30-60 km/h)", 30.0, 60.0),
        ("High speed corners (60+ km/h)", 60.0, math.inf),
    ]

    for label, lower, upper in corner_bins:
        mask = [
            is_corner and lower <= speed < upper
            for is_corner, speed in zip(corner_mask, avg_speed)
        ]
        coverage = mask_distance(grid, mask)
        basis = f"mean speed over {format_distance(coverage)}"
        rows.append(
            make_row(
                label,
                run_a.label,
                run_b.label,
                mask_mean(speed_a, mask),
                mask_mean(speed_b, mask),
                "km/h",
                basis,
                higher_is_better=True,
                tolerance=0.2,
                precision=2,
            )
        )

    accel_a_g: Optional[float] = None
    accel_b_g: Optional[float] = None
    accel_basis = "insufficient longitudinal accel data"
    if ax_a is not None and ax_b is not None:
        avg_ax = [(a + b) / 2.0 for a, b in zip(ax_a, ax_b)]
        avg_throttle = combine_optional_mean(throttle_a, throttle_b, 100.0)
        avg_brake = combine_optional_mean(brake_a, brake_b, 0.0)
        accel_mask = []
        for idx, ax in enumerate(avg_ax):
            throttle_ok = not avg_throttle or avg_throttle[idx] >= throttle_threshold
            brake_ok = not avg_brake or avg_brake[idx] < brake_threshold
            accel_mask.append(
                (not corner_mask[idx])
                and avg_speed[idx] >= 5.0
                and ax > 0.25
                and throttle_ok
                and brake_ok
            )
        accel_a_g = mask_mean([max(value, 0.0) / G_MPS2 for value in ax_a], accel_mask)
        accel_b_g = mask_mean([max(value, 0.0) / G_MPS2 for value in ax_b], accel_mask)
        accel_basis = f"mean positive ax over {format_distance(mask_distance(grid, accel_mask))}"

    rows.append(
        make_row(
            "Acceleration",
            run_a.label,
            run_b.label,
            accel_a_g,
            accel_b_g,
            "g",
            accel_basis,
            higher_is_better=True,
            tolerance=0.02,
            precision=3,
        )
    )

    braking_a_g: Optional[float] = None
    braking_b_g: Optional[float] = None
    braking_basis = "insufficient longitudinal accel data"
    if ax_a is not None and ax_b is not None:
        avg_ax = [(a + b) / 2.0 for a, b in zip(ax_a, ax_b)]
        avg_brake = combine_optional_mean(brake_a, brake_b, 0.0)
        has_brake_channel = brake_a is not None or brake_b is not None
        braking_mask = []
        for idx, ax in enumerate(avg_ax):
            if has_brake_channel:
                braking_mask.append(bool(avg_brake) and avg_brake[idx] >= brake_threshold)
            else:
                braking_mask.append(ax < -0.5)
        braking_a_g = mask_mean([max(-value, 0.0) / G_MPS2 for value in ax_a], braking_mask)
        braking_b_g = mask_mean([max(-value, 0.0) / G_MPS2 for value in ax_b], braking_mask)
        braking_basis = f"mean decel over {format_distance(mask_distance(grid, braking_mask))}"

    rows.append(
        make_row(
            "Braking",
            run_a.label,
            run_b.label,
            braking_a_g,
            braking_b_g,
            "g",
            braking_basis,
            higher_is_better=True,
            tolerance=0.02,
            precision=3,
        )
    )

    rows.append(
        make_row(
            "Top speed",
            run_a.label,
            run_b.label,
            max_finite(speed_a),
            max_finite(speed_b),
            "km/h",
            "max speed in common distance range",
            higher_is_better=True,
            tolerance=0.5,
            precision=2,
        )
    )

    if time_a is not None and time_b is not None:
        elapsed_a = time_a[-1] - time_a[0]
        elapsed_b = time_b[-1] - time_b[0]
    else:
        elapsed_a = None
        elapsed_b = None

    rows.append(
        make_row(
            "Elapsed time",
            run_a.label,
            run_b.label,
            elapsed_a,
            elapsed_b,
            "s",
            "lower is better over common distance",
            higher_is_better=False,
            tolerance=0.01,
            precision=3,
        )
    )

    report_lines = [
        "# Sim Run Comparison",
        "",
        f"- Car A: {run_a.label} ({run_a.path})",
        f"- Car B: {run_b.label} ({run_b.path})",
        f"- Track: {track.label if track else 'not provided'}",
        f"- Curvature source: {curvature_source}",
        f"- Common distance: {start:.2f} m to {end:.2f} m "
        f"({len(grid)} samples at about {distance_step:g} m)",
    ]
    if curvature_warning:
        report_lines.append(f"- Track note: {curvature_warning}")

    report_lines.extend(
        [
            "",
            "| Area | Stronger | A | B | Delta (B - A) | Basis |",
            "| --- | --- | ---: | ---: | ---: | --- |",
        ]
    )
    for row in rows:
        report_lines.append(
            f"| {row.area} | {row.stronger} | {row.a_value} | {row.b_value} | "
            f"{row.delta} | {row.basis} |"
        )

    report_lines.extend(
        [
            "",
            "Notes:",
            f"- Corner rows use samples where abs(curvature) >= {corner_curvature_threshold:g} 1/m.",
            "- Corner speed bins are based on the average of both cars' speed at each station.",
            f"- Acceleration uses non-corner samples above 5 km/h with positive ax, throttle >= "
            f"{throttle_threshold:g}%, and brake < {brake_threshold:g}% when those channels exist.",
            f"- Braking uses samples with brake >= {brake_threshold:g}% when brake channels exist; "
            "otherwise it falls back to average ax < -0.5 m/s^2.",
        ]
    )
    return "\n".join(report_lines) + "\n"


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Compare two simulated car runs from MoTeC-style CSV telemetry.",
    )
    parser.add_argument("car_a_csv", type=Path, help="First telemetry CSV.")
    parser.add_argument("car_b_csv", type=Path, help="Second telemetry CSV.")
    parser.add_argument(
        "--track",
        help=(
            "Associated track file or name. CSV tracks with station/curvature are read directly; "
            ".mat tracks require scipy. If omitted or unreadable, telemetry curvature is used."
        ),
    )
    parser.add_argument("--label-a", help="Display label for the first car.")
    parser.add_argument("--label-b", help="Display label for the second car.")
    parser.add_argument(
        "--output",
        "-o",
        type=Path,
        help="Optional Markdown report path. The report is always printed to stdout.",
    )
    parser.add_argument(
        "--distance-step",
        type=float,
        default=0.25,
        help="Distance-grid step in metres for comparing the two runs. Default: 0.25.",
    )
    parser.add_argument(
        "--corner-curvature-threshold",
        type=float,
        default=0.01,
        help="Absolute curvature threshold in 1/m used to identify corners. Default: 0.01.",
    )
    parser.add_argument(
        "--throttle-threshold",
        type=float,
        default=50.0,
        help="Throttle percentage threshold for acceleration-zone detection. Default: 50.",
    )
    parser.add_argument(
        "--brake-threshold",
        type=float,
        default=5.0,
        help="Brake percentage threshold for braking-zone detection. Default: 5.",
    )
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)

    try:
        run_a = read_telemetry_csv(args.car_a_csv, args.label_a)
        run_b = read_telemetry_csv(args.car_b_csv, args.label_b)
        track = load_track_curvature(args.track)
        report = compare_runs(
            run_a,
            run_b,
            track,
            distance_step=args.distance_step,
            corner_curvature_threshold=args.corner_curvature_threshold,
            throttle_threshold=args.throttle_threshold,
            brake_threshold=args.brake_threshold,
        )
    except Exception as exc:
        parser.exit(2, f"compare_sim_runs: error: {exc}\n")

    print(report, end="")
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(report, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
