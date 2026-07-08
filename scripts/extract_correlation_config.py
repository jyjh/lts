#!/usr/bin/env python3
"""Estimate replay channel timing offsets and write a correlation config JSON."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np


SCHEMA = "lts.correlation.config.v1"


def read_replay_csv(path: Path) -> dict[str, np.ndarray]:
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"{path} has no header row")

        columns = {name: [] for name in reader.fieldnames}
        for row in reader:
            for name in columns:
                columns[name].append(parse_float(row.get(name, "")))

    return {name: np.asarray(values, dtype=float) for name, values in columns.items()}


def parse_float(value: str | None) -> float:
    if value is None:
        return math.nan
    text = str(value).strip()
    if not text:
        return math.nan
    try:
        return float(text)
    except ValueError:
        return math.nan


def replay_time(data: dict[str, np.ndarray]) -> np.ndarray:
    if "time_s" not in data:
        raise ValueError("Replay CSV must contain time_s")
    time = np.asarray(data["time_s"], dtype=float)
    finite = time[np.isfinite(time)]
    if finite.size < 2:
        raise ValueError("Replay CSV must contain at least two finite time_s samples")
    return time - finite[0]


def estimate_pack_power_advance(
    data: dict[str, np.ndarray],
    max_advance_s: float = 0.4,
    step_s: float = 0.005,
) -> dict:
    time = replay_time(data)
    voltage = data.get("pack_voltage_v")
    current = data.get("pack_current_a")
    if voltage is None or current is None:
        return unavailable("missing pack_voltage_v or pack_current_a")

    command = data.get("motor_torque_command_nm")
    source = "negative_motor_torque_command"
    if command is None or np.count_nonzero(np.isfinite(command)) < 3:
        command = data.get("regen_torque_nm")
        source = "regen_torque_command"
    if command is None:
        return unavailable("missing motor_torque_command_nm and regen_torque_nm")

    demand_nm = np.maximum(0.0, -np.asarray(command, dtype=float))
    pack_power_kw = np.asarray(voltage, dtype=float) * np.asarray(current, dtype=float) / 1000.0
    charging_kw = np.maximum(0.0, -pack_power_kw)
    active = (demand_nm > 1.0) | (charging_kw > 0.1)
    result = sweep_alignment(
        time,
        reference=demand_nm,
        shifted_signal=charging_kw,
        active=active,
        max_advance_s=max_advance_s,
        step_s=step_s,
        objective="rmse",
    )
    result.update(
        {
            "source": source,
            "reference": "negative torque demand [Nm]",
            "shiftedChannel": "negative pack power [kW]",
            "meaning": "Positive advance pulls pack_voltage_v and pack_current_a earlier.",
        }
    )
    return result


def estimate_gps_advance(
    data: dict[str, np.ndarray],
    max_advance_s: float = 1.0,
    step_s: float = 0.01,
    smoothing_s: float = 0.15,
) -> dict:
    time = replay_time(data)
    yaw_rate = data.get("yaw_rate_radps")
    gps_course = data.get("gps_course_rad")
    if yaw_rate is not None and gps_course is not None:
        math_heading = unwrap_finite(np.pi / 2.0 - np.asarray(gps_course, dtype=float))
        gps_rate = finite_gradient(math_heading, time)
        yaw_rate_s = smooth_by_time(np.asarray(yaw_rate, dtype=float), time, smoothing_s)
        gps_rate_s = smooth_by_time(gps_rate, time, smoothing_s)
        active = (np.abs(yaw_rate_s) > 0.05) | (np.abs(gps_rate_s) > 0.05)
        result = sweep_alignment(
            time,
            reference=yaw_rate_s,
            shifted_signal=gps_rate_s,
            active=active,
            max_advance_s=max_advance_s,
            step_s=step_s,
            objective="abs_corr",
        )
        result.update(
            {
                "source": "gps_course_rate_vs_yaw_rate",
                "reference": "yaw_rate_radps",
                "shiftedChannel": "gps_course_rad derivative",
                "meaning": "Positive advance pulls GPS position/course channels earlier.",
            }
        )
        return result

    speed = data.get("speed_mps")
    lat = data.get("gps_lat_deg")
    lon = data.get("gps_lon_deg")
    if speed is None or lat is None or lon is None:
        return unavailable("missing GPS course/yaw-rate and GPS position/speed fallback")

    gps_speed = gps_speed_from_lat_lon(time, lat, lon)
    gps_speed = smooth_by_time(gps_speed, time, smoothing_s)
    speed_s = smooth_by_time(np.asarray(speed, dtype=float), time, smoothing_s)
    active = (speed_s > 2.0) | (gps_speed > 2.0)
    result = sweep_alignment(
        time,
        reference=speed_s,
        shifted_signal=gps_speed,
        active=active,
        max_advance_s=max_advance_s,
        step_s=step_s,
        objective="corr",
    )
    result.update(
        {
            "source": "gps_position_speed_vs_speed_mps",
            "reference": "speed_mps",
            "shiftedChannel": "GPS lat/lon path speed",
            "meaning": "Positive advance pulls GPS position/course channels earlier.",
        }
    )
    return result


def sweep_alignment(
    time: np.ndarray,
    reference: np.ndarray,
    shifted_signal: np.ndarray,
    active: np.ndarray,
    max_advance_s: float,
    step_s: float,
    objective: str,
) -> dict:
    time = np.asarray(time, dtype=float)
    reference = np.asarray(reference, dtype=float)
    shifted_signal = np.asarray(shifted_signal, dtype=float)
    active = np.asarray(active, dtype=bool)
    if step_s <= 0:
        raise ValueError("Alignment step must be positive")
    if max_advance_s < 0:
        raise ValueError("Maximum advance must be nonnegative")

    advances = np.arange(-max_advance_s, max_advance_s + 0.5 * step_s, step_s)
    rows = []
    for advance_s in advances:
        shifted = interpolate_shift(time, shifted_signal, advance_s)
        valid = active & np.isfinite(reference) & np.isfinite(shifted)
        sample_count = int(np.count_nonzero(valid))
        if sample_count < 20:
            rows.append(metric_row(advance_s, sample_count))
            continue

        ref = reference[valid]
        sig = shifted[valid]
        gain = fit_gain(sig, ref)
        residual = ref - gain * sig
        rmse = float(np.sqrt(np.mean(residual * residual)))
        corr = finite_corr(ref, sig)
        rows.append(metric_row(advance_s, sample_count, gain, rmse, corr))

    candidates = [row for row in rows if row["sampleCount"] >= 20 and row["correlation"] is not None]
    if not candidates:
        return unavailable("insufficient active samples")

    if objective == "rmse":
        best = min(candidates, key=lambda row: (row["scaledRmse"], abs(row["advanceS"])))
    elif objective == "abs_corr":
        best = max(candidates, key=lambda row: (abs(row["correlation"]), -abs(row["advanceS"])))
    elif objective == "corr":
        best = max(candidates, key=lambda row: (row["correlation"], -abs(row["advanceS"])))
    else:
        raise ValueError(f"Unsupported alignment objective: {objective}")

    return {
        "status": "ok",
        "advanceS": clean_float(best["advanceS"]),
        "sampleCount": int(best["sampleCount"]),
        "gain": clean_float(best["gain"]),
        "scaledRmse": clean_float(best["scaledRmse"]),
        "correlation": clean_float(best["correlation"]),
        "quality": estimate_quality(best["correlation"]),
        "objective": objective,
        "search": {
            "minAdvanceS": clean_float(-max_advance_s),
            "maxAdvanceS": clean_float(max_advance_s),
            "stepS": clean_float(step_s),
        },
    }


def estimate_quality(correlation: float | None) -> str:
    if correlation is None or not np.isfinite(correlation):
        return "unknown"
    abs_corr = abs(float(correlation))
    if abs_corr >= 0.65:
        return "high"
    if abs_corr >= 0.35:
        return "medium"
    return "low"


def unavailable(reason: str) -> dict:
    return {"status": "unavailable", "reason": reason, "advanceS": None}


def metric_row(
    advance_s: float,
    sample_count: int,
    gain: float | None = None,
    rmse: float | None = None,
    corr: float | None = None,
) -> dict:
    return {
        "advanceS": float(advance_s),
        "sampleCount": int(sample_count),
        "gain": gain,
        "scaledRmse": rmse,
        "correlation": corr,
    }


def interpolate_shift(time: np.ndarray, values: np.ndarray, advance_s: float) -> np.ndarray:
    time = np.asarray(time, dtype=float)
    values = np.asarray(values, dtype=float)
    keep = np.isfinite(time) & np.isfinite(values)
    if np.count_nonzero(keep) < 2:
        return np.full(time.shape, np.nan)
    source_time = time[keep]
    source_values = values[keep]
    unique_time, unique_idx = np.unique(source_time, return_index=True)
    unique_values = source_values[unique_idx]
    query = time + advance_s
    shifted = np.interp(query, unique_time, unique_values)
    shifted[(query < unique_time[0]) | (query > unique_time[-1])] = np.nan
    return shifted


def fit_gain(signal: np.ndarray, reference: np.ndarray) -> float:
    denom = float(np.dot(signal, signal))
    if denom <= np.finfo(float).eps:
        return 0.0
    return float(np.dot(signal, reference) / denom)


def finite_corr(a: np.ndarray, b: np.ndarray) -> float | None:
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    keep = np.isfinite(a) & np.isfinite(b)
    if np.count_nonzero(keep) < 2:
        return None
    a = a[keep]
    b = b[keep]
    if np.std(a) <= np.finfo(float).eps or np.std(b) <= np.finfo(float).eps:
        return None
    return float(np.corrcoef(a, b)[0, 1])


def unwrap_finite(values: np.ndarray) -> np.ndarray:
    values = np.asarray(values, dtype=float).copy()
    finite = np.isfinite(values)
    if np.any(finite):
        values[finite] = np.unwrap(values[finite])
    return values


def finite_gradient(values: np.ndarray, time: np.ndarray) -> np.ndarray:
    values = np.asarray(values, dtype=float)
    time = np.asarray(time, dtype=float)
    keep = np.isfinite(values) & np.isfinite(time)
    output = np.full(values.shape, np.nan)
    if np.count_nonzero(keep) < 3:
        return output
    output[keep] = np.gradient(values[keep], time[keep])
    return output


def smooth_by_time(values: np.ndarray, time: np.ndarray, window_s: float) -> np.ndarray:
    values = np.asarray(values, dtype=float)
    finite_dt = np.diff(time[np.isfinite(time)])
    finite_dt = finite_dt[finite_dt > 0]
    if finite_dt.size == 0 or window_s <= 0:
        return values
    sample_count = max(1, int(round(window_s / float(np.median(finite_dt)))))
    if sample_count <= 1:
        return values
    kernel = np.ones(sample_count, dtype=float)
    finite = np.isfinite(values).astype(float)
    filled = np.where(np.isfinite(values), values, 0.0)
    numerator = np.convolve(filled, kernel, mode="same")
    denominator = np.convolve(finite, kernel, mode="same")
    output = numerator / np.maximum(denominator, np.finfo(float).eps)
    output[denominator <= 0] = np.nan
    return output


def gps_speed_from_lat_lon(time: np.ndarray, lat_deg: np.ndarray, lon_deg: np.ndarray) -> np.ndarray:
    lat = np.asarray(lat_deg, dtype=float)
    lon = np.asarray(lon_deg, dtype=float)
    keep = np.isfinite(lat) & np.isfinite(lon)
    if np.count_nonzero(keep) < 3:
        return np.full(time.shape, np.nan)
    lat0 = float(lat[keep][0])
    lon0 = float(lon[keep][0])
    earth_radius_m = 6378137.0
    east = np.deg2rad(lon - lon0) * earth_radius_m * math.cos(math.radians(lat0))
    north = np.deg2rad(lat - lat0) * earth_radius_m
    ve = finite_gradient(east, time)
    vn = finite_gradient(north, time)
    return np.hypot(ve, vn)


def clean_float(value: float | None) -> float | None:
    if value is None or not np.isfinite(value):
        return None
    return round(float(value), 6)


def build_config(replay_csv: Path, pack: dict, gps: dict) -> dict:
    pack_advance = pack.get("advanceS")
    gps_advance = gps.get("advanceS")
    return {
        "schema": SCHEMA,
        "sourceReplayCsv": str(replay_csv),
        "offsets": {
            "PackPowerAdvanceS": pack_advance,
            "GpsAdvanceS": gps_advance,
        },
        "runCorrelationOptions": {
            "PackPowerAdvanceS": pack_advance,
        },
        "plotCorrelationPositionOverlayOptions": {
            "RawTimeOffsetS": gps_advance,
        },
        "estimates": {
            "PackPowerAdvanceS": pack,
            "GpsAdvanceS": gps,
        },
    }


def default_output_path(replay_csv: Path) -> Path:
    stem = replay_csv.stem
    if stem.endswith("_replay"):
        stem = stem[: -len("_replay")]
    return replay_csv.with_name(f"{stem}_correlation_config.json")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Estimate replay timing offsets and write a secondary correlation config JSON."
    )
    parser.add_argument("replay_csv", nargs="?", help="Normalized replay CSV from extract_motec_lap.py")
    parser.add_argument("--input", dest="input_csv", help="Normalized replay CSV")
    parser.add_argument("--output", help="Output JSON path; defaults beside the replay CSV")
    parser.add_argument("--pack-max-advance", type=float, default=0.4)
    parser.add_argument("--pack-step", type=float, default=0.005)
    parser.add_argument("--gps-max-advance", type=float, default=1.0)
    parser.add_argument("--gps-step", type=float, default=0.01)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    input_text = args.input_csv or args.replay_csv
    if not input_text:
        raise SystemExit("Provide a replay CSV path or --input")

    replay_csv = Path(input_text)
    data = read_replay_csv(replay_csv)
    pack = estimate_pack_power_advance(data, args.pack_max_advance, args.pack_step)
    gps = estimate_gps_advance(data, args.gps_max_advance, args.gps_step)
    config = build_config(replay_csv, pack, gps)

    output_path = Path(args.output) if args.output else default_output_path(replay_csv)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as handle:
        json.dump(config, handle, indent=2)
        handle.write("\n")

    print(f"Correlation config written: {output_path}")
    print(f"PackPowerAdvanceS: {pack.get('advanceS')} ({pack.get('status')})")
    print(f"GpsAdvanceS: {gps.get('advanceS')} ({gps.get('status')})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
