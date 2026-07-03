#!/usr/bin/env python3
"""Extract a real MoTeC lap into the simulator replay CSV contract."""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import sys
from pathlib import Path

import numpy as np


REPLAY_COLUMNS = [
    "time_s",
    "distance_m",
    "throttle_ratio",
    "brake_ratio",
    "steer_rad",
    "speed_mps",
    "yaw_rad",
    "yaw_rate_radps",
    "x_m",
    "y_m",
    "gps_lat_deg",
    "gps_lon_deg",
    "gps_course_rad",
    "lat_accel_g",
    "long_accel_g",
]


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def load_ldparser():
    parser_dir = repo_root() / "external" / "MotecLogGenerator" / "ldparser"
    sys.path.insert(0, str(parser_dir))
    from ldparser import ldData  # pylint: disable=import-error,import-outside-toplevel

    return ldData


def normalize_name(value: str) -> str:
    return re.sub(r"[^a-z0-9]", "", str(value).lower())


def load_channel_map(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as handle:
        channel_map = json.load(handle)

    if "channels" not in channel_map:
        raise ValueError(f"{path} must contain a 'channels' object")

    return channel_map


def channel_lookup(data) -> dict:
    lookup = {}
    for channel in data.channs:
        lookup.setdefault(normalize_name(channel.name), channel)
    return lookup


def find_channel(data, spec: dict):
    lookup = channel_lookup(data)
    for name in spec.get("names", []):
        channel = lookup.get(normalize_name(name))
        if channel is not None:
            return channel
    return None


def direct_source_specs(spec: dict):
    sources = spec.get("sources")
    if not sources:
        yield spec
        return

    inherited = {
        key: value
        for key, value in spec.items()
        if key not in {"sources", "derive", "names", "source_unit_scale"}
    }
    inherited_unit_scale = spec.get("source_unit_scale", {})
    for source in sources:
        merged = dict(inherited)
        merged.update(source)
        if inherited_unit_scale or source.get("source_unit_scale"):
            unit_scale_map = dict(inherited_unit_scale)
            unit_scale_map.update(source.get("source_unit_scale", {}))
            merged["source_unit_scale"] = unit_scale_map
        yield merged


def channel_aliases(spec: dict) -> list[str]:
    aliases = list(spec.get("names", []))
    for source in spec.get("sources", []):
        aliases.extend(source.get("names", []))
    return aliases


def unit_scale(spec: dict, unit: str) -> float:
    scale = float(spec.get("scale", 1.0))
    by_unit = spec.get("source_unit_scale", {})
    unit = str(unit).strip()
    if unit in by_unit:
        scale *= float(by_unit[unit])
        return scale

    unit_lower = unit.lower()
    for known_unit, known_scale in by_unit.items():
        if str(known_unit).lower() == unit_lower:
            scale *= float(known_scale)
            break
    return scale


def channel_time(channel) -> np.ndarray:
    freq = max(float(channel.freq), 1.0)
    return np.arange(channel.data_len, dtype=float) / freq


def extract_raw_signal(data, output_name: str, spec: dict):
    signal = extract_direct_signal(data, output_name, spec, required=False)
    if signal is not None:
        return signal

    derive_spec = spec.get("derive")
    if derive_spec:
        return derive_raw_signal(data, output_name, spec, derive_spec)

    if bool(spec.get("required", False)):
        aliases = ", ".join(spec.get("names", []))
        raise ValueError(f"Missing required channel for {output_name}: {aliases}")

    return None


def extract_direct_signal(data, output_name: str, spec: dict, required: bool):
    channel = None
    matched_spec = None
    for source_spec in direct_source_specs(spec):
        channel = find_channel(data, source_spec)
        if channel is not None:
            matched_spec = source_spec
            break

    if channel is None or matched_spec is None:
        if required:
            aliases = ", ".join(channel_aliases(spec))
            raise ValueError(f"Missing required channel for {output_name}: {aliases}")
        return None

    values = np.asarray(channel.data, dtype=float)
    scale = unit_scale(matched_spec, getattr(channel, "unit", ""))
    offset = float(matched_spec.get("offset", 0.0))
    values = values * scale + offset

    if "clamp" in spec:
        lo, hi = spec["clamp"]
        values = np.clip(values, float(lo), float(hi))

    signal = {
        "name": channel.name,
        "unit": getattr(channel, "unit", ""),
        "frequency_hz": float(channel.freq),
        "sample_count": int(channel.data_len),
        "scale_applied": scale,
        "offset_applied": offset,
        "source_label": matched_spec.get("label", ""),
        "time": channel_time(channel),
        "values": values,
    }
    signal.update(signal_stats(values))
    return signal


def derive_raw_signal(data, output_name: str, spec: dict, derive_spec: dict):
    method = derive_spec.get("method", "")
    if method == "brake_pressure":
        return derive_brake_ratio_from_pressure(data, output_name, spec, derive_spec)

    raise ValueError(f"Unsupported derive method for {output_name}: {method}")


def derive_brake_ratio_from_pressure(data, output_name: str, spec: dict, derive_spec: dict):
    front = extract_direct_signal(
        data,
        f"{output_name}.front",
        derive_spec.get("front", {}),
        required=True,
    )
    rear = extract_direct_signal(
        data,
        f"{output_name}.rear",
        derive_spec.get("rear", {}),
        required=True,
    )

    freq = max(front["frequency_hz"], rear["frequency_hz"], 1.0)
    duration = min(float(front["time"][-1]), float(rear["time"][-1]))
    if duration <= 0:
        raise ValueError("Brake pressure channels have no positive duration")

    sample_count = max(2, int(math.floor(duration * freq)) + 1)
    time = np.arange(sample_count, dtype=float) / freq
    time = time[time <= duration + 1e-12]

    front_pressure = resample_signal(front, time, 0.0)
    rear_pressure = resample_signal(rear, time, 0.0)

    front_pressure = zero_pressure(front_pressure, derive_spec, "front")
    rear_pressure = zero_pressure(rear_pressure, derive_spec, "rear")

    front_ratio = pressure_to_ratio(front_pressure, derive_spec, "front")
    rear_ratio = pressure_to_ratio(rear_pressure, derive_spec, "rear")
    ratio = combine_brake_ratios(front_ratio, rear_ratio, derive_spec)

    if "clamp" in spec:
        lo, hi = spec["clamp"]
        ratio = np.clip(ratio, float(lo), float(hi))

    signal = {
        "name": f"{front['name']} + {rear['name']}",
        "unit": "ratio",
        "frequency_hz": float(freq),
        "sample_count": int(len(time)),
        "scale_applied": 1.0,
        "offset_applied": 0.0,
        "source": "derived",
        "derive_method": "brake_pressure",
        "combine": derive_spec.get("combine", "max"),
        "components": {
            "front": signal_manifest(front),
            "rear": signal_manifest(rear),
        },
        "time": time,
        "values": ratio,
    }
    signal.update(signal_stats(ratio))
    return signal


def zero_pressure(values: np.ndarray, derive_spec: dict, axle: str) -> np.ndarray:
    values = np.asarray(values, dtype=float)
    zero = derive_spec.get(f"{axle}_zero_bar", derive_spec.get("zero_bar", 0.0))
    if zero is None:
        zero = 0.0
    values = values - float(zero)

    percentile = derive_spec.get("auto_zero_percentile")
    if percentile is not None:
        finite = values[np.isfinite(values)]
        if len(finite):
            values = values - float(np.percentile(finite, float(percentile)))

    return np.maximum(values, 0.0)


def pressure_to_ratio(values: np.ndarray, derive_spec: dict, axle: str) -> np.ndarray:
    full_scale = derive_spec.get(
        f"{axle}_full_scale_bar",
        derive_spec.get("full_scale_bar"),
    )
    if full_scale is None:
        if not bool(derive_spec.get("auto_scale_if_missing", True)):
            raise ValueError(
                f"Brake pressure derivation requires {axle}_full_scale_bar "
                "or full_scale_bar when auto_scale_if_missing is false"
            )
        finite = values[np.isfinite(values)]
        full_scale = float(np.max(finite)) if len(finite) else 1.0

    full_scale = max(float(full_scale), np.finfo(float).eps)
    return values / full_scale


def combine_brake_ratios(front_ratio: np.ndarray, rear_ratio: np.ndarray, derive_spec: dict):
    combine = str(derive_spec.get("combine", "max")).lower()
    if combine == "front":
        return front_ratio
    if combine == "rear":
        return rear_ratio
    if combine == "mean":
        return 0.5 * (front_ratio + rear_ratio)
    if combine == "weighted_mean":
        front_weight = float(derive_spec.get("front_weight", 0.5))
        rear_weight = float(derive_spec.get("rear_weight", 1.0 - front_weight))
        denom = max(front_weight + rear_weight, np.finfo(float).eps)
        return (front_weight * front_ratio + rear_weight * rear_ratio) / denom
    if combine == "max":
        return np.maximum(front_ratio, rear_ratio)

    raise ValueError(f"Unsupported brake pressure combine method: {combine}")


def signal_manifest(signal: dict) -> dict:
    return {
        "name": signal["name"],
        "unit": signal["unit"],
        "frequency_hz": signal["frequency_hz"],
        "sample_count": signal["sample_count"],
        "scale_applied": signal["scale_applied"],
        "offset_applied": signal["offset_applied"],
        "source_label": signal.get("source_label", ""),
        "min_value": signal.get("min_value"),
        "max_value": signal.get("max_value"),
    }


def signal_stats(values: np.ndarray) -> dict:
    finite = np.asarray(values, dtype=float)
    finite = finite[np.isfinite(finite)]
    if len(finite) == 0:
        return {"min_value": None, "max_value": None}
    return {"min_value": float(np.min(finite)), "max_value": float(np.max(finite))}


def choose_output_frequency(args, channel_map: dict, raw_signals: dict) -> float:
    if args.frequency:
        return float(args.frequency)
    if "sample_frequency_hz" in channel_map:
        return float(channel_map["sample_frequency_hz"])

    freqs = [
        sig["frequency_hz"]
        for sig in raw_signals.values()
        if sig is not None and sig["frequency_hz"] > 0
    ]
    return max(freqs) if freqs else 1000.0


def signal_duration(raw_signals: dict, required_names: set[str]) -> float:
    durations = []
    for name, signal in raw_signals.items():
        if signal is None:
            continue
        if name in required_names and len(signal["time"]) >= 2:
            durations.append(float(signal["time"][-1]))

    if not durations:
        raise ValueError("Could not determine lap duration from required channels")

    duration = min(durations)
    if duration <= 0:
        raise ValueError("Selected lap has no positive duration")
    return duration


def resample_signal(signal, time_out: np.ndarray, default_value: float) -> np.ndarray:
    if signal is None:
        return np.full(time_out.shape, default_value, dtype=float)

    time = signal["time"]
    values = signal["values"]
    finite = np.isfinite(time) & np.isfinite(values)
    time = time[finite]
    values = values[finite]
    if len(time) == 0:
        return np.full(time_out.shape, default_value, dtype=float)
    if len(time) == 1:
        return np.full(time_out.shape, values[0], dtype=float)

    unique_time, unique_idx = np.unique(time, return_index=True)
    unique_values = values[unique_idx]
    return np.interp(time_out, unique_time, unique_values)


def integrate_distance(time_s: np.ndarray, speed_mps: np.ndarray) -> np.ndarray:
    distance = np.zeros_like(time_s)
    if len(time_s) < 2:
        return distance

    dt = np.diff(time_s)
    distance[1:] = np.cumsum(0.5 * (speed_mps[:-1] + speed_mps[1:]) * dt)
    return distance


def write_csv(path: Path, table: dict[str, np.ndarray]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(REPLAY_COLUMNS)
        row_count = len(table["time_s"])
        for idx in range(row_count):
            writer.writerow([format_value(table[column][idx]) for column in REPLAY_COLUMNS])


def format_value(value: float) -> str:
    if value is None or not math.isfinite(float(value)):
        return "NaN"
    return f"{float(value):.12g}"


def default_from_spec(spec: dict) -> float:
    value = spec.get("default", np.nan)
    return np.nan if value is None else float(value)


def write_manifest(path: Path, manifest: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(manifest, handle, indent=2, sort_keys=True)
        handle.write("\n")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, help="Input MoTeC .ld file")
    parser.add_argument("--output", required=True, help="Normalized replay CSV path")
    parser.add_argument("--manifest", help="Optional extraction manifest JSON path")
    parser.add_argument("--channel-map", required=True, help="Channel map JSON path")
    parser.add_argument("--laps", help="1-based lap or inclusive range, e.g. 4 or 4-5")
    parser.add_argument("--ldx", help="Optional .ldx sidecar path for lap markers")
    parser.add_argument("--frequency", type=float, help="Output sample frequency [Hz]")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    input_file = Path(args.input)
    output_file = Path(args.output)
    channel_map_file = Path(args.channel_map)
    manifest_file = Path(args.manifest) if args.manifest else output_file.with_suffix(".manifest.json")

    ld_data = load_ldparser().fromfile(
        str(input_file),
        laps=args.laps,
        ldx_file=args.ldx,
    )
    channel_map = load_channel_map(channel_map_file)

    raw_signals = {}
    required_names = set()
    for output_name, spec in channel_map["channels"].items():
        raw_signals[output_name] = extract_raw_signal(ld_data, output_name, spec)
        if bool(spec.get("required", False)):
            required_names.add(output_name)

    output_frequency = choose_output_frequency(args, channel_map, raw_signals)
    duration = signal_duration(raw_signals, required_names)
    sample_count = max(2, int(math.floor(duration * output_frequency)) + 1)
    time_out = np.arange(sample_count, dtype=float) / output_frequency
    time_out = time_out[time_out <= duration + 1e-12]

    table = {"time_s": time_out}
    for column in REPLAY_COLUMNS:
        if column == "time_s":
            continue
        spec = channel_map["channels"].get(column, {})
        default_value = default_from_spec(spec)
        table[column] = resample_signal(raw_signals.get(column), time_out, default_value)

    if raw_signals.get("distance_m") is None or np.all(~np.isfinite(table["distance_m"])):
        table["distance_m"] = integrate_distance(table["time_s"], table["speed_mps"])
    else:
        table["distance_m"] = table["distance_m"] - table["distance_m"][0]

    write_csv(output_file, table)

    manifest = {
        "input_file": str(input_file),
        "output_file": str(output_file),
        "channel_map": str(channel_map_file),
        "laps": args.laps,
        "ldx_file": args.ldx,
        "sample_frequency_hz": output_frequency,
        "sample_count": int(len(time_out)),
        "duration_s": float(time_out[-1]) if len(time_out) else 0.0,
        "channels": {
            name: None
            if signal is None
            else {
                key: value
                for key, value in signal.items()
                if key not in {"time", "values"}
            }
            for name, signal in raw_signals.items()
        },
    }
    write_manifest(manifest_file, manifest)
    print(f"Wrote replay CSV: {output_file}")
    print(f"Wrote manifest: {manifest_file}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
