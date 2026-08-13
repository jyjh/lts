#!/usr/bin/env python3
"""Extract a real MoTeC lap into the simulator replay CSV contract."""

from __future__ import annotations

import argparse
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
    "brake_pressure_front_bar",
    "brake_pressure_rear_bar",
    "regen_torque_nm",
    "motor_torque_command_nm",
    "motor_torque_delivered_nm",
    "motor_rpm",
    "pack_voltage_v",
    "pack_current_a",
    "steer_rad",
    "speed_mps",
    "wheel_speed_fl_mps",
    "wheel_speed_fr_mps",
    "wheel_speed_rl_mps",
    "wheel_speed_rr_mps",
    "vx_mps",
    "vy_mps",
    "body_slip_rad",
    "yaw_rad",
    "yaw_rate_radps",
    "x_m",
    "y_m",
    "gps_lat_deg",
    "gps_lon_deg",
    "gps_course_rad",
    "front_lat_accel_g",
    "rear_lat_accel_g",
    "lat_accel_g",
    "front_long_accel_g",
    "rear_long_accel_g",
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
    cached = getattr(data, "_lts_channel_lookup", None)
    if cached is not None:
        return cached

    lookup = {}
    for channel in data.channs:
        lookup.setdefault(normalize_name(channel.name), channel)
    setattr(data, "_lts_channel_lookup", lookup)
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
    derive_spec = spec.get("derive")
    if bool(spec.get("prefer_derive", False)) and derive_spec:
        signal = derive_raw_signal(data, output_name, spec, derive_spec)
        if signal is not None:
            return signal

    signal = extract_direct_signal(data, output_name, spec, required=False)
    if signal is not None:
        return signal

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
    values, transform = apply_transform(values, matched_spec)
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
        "transform_applied": transform,
        "source_label": matched_spec.get("label", ""),
        "time": channel_time(channel),
        "values": values,
    }
    signal.update(signal_stats(values))
    return signal


def apply_transform(values: np.ndarray, spec: dict) -> tuple[np.ndarray, str]:
    transform = str(spec.get("transform", "")).strip()
    if not transform:
        return values, ""

    if transform == "uint16_to_int16":
        return uint16_to_int16(values), transform

    if transform == "unsigned_negative_torque":
        decoded = uint16_to_int16(values)
        return np.where(decoded < 0.0, decoded, -np.abs(decoded)), transform

    raise ValueError(f"Unsupported transform: {transform}")


def uint16_to_int16(values: np.ndarray) -> np.ndarray:
    decoded = np.asarray(values, dtype=float).copy()
    finite_unsigned = np.isfinite(decoded) & (decoded >= 0.0) & (decoded <= 65536.0)
    decoded[finite_unsigned] = np.mod(decoded[finite_unsigned], 65536.0)
    finite_wrapped = np.isfinite(decoded) & (decoded > 32767.0) & (decoded <= 65535.0)
    decoded[finite_wrapped] = decoded[finite_wrapped] - 65536.0
    return decoded


def derive_raw_signal(data, output_name: str, spec: dict, derive_spec: dict):
    method = derive_spec.get("method", "")
    if method == "brake_pressure":
        return derive_brake_ratio_from_pressure(data, output_name, spec, derive_spec)
    if method == "wheel_speed_median":
        return derive_speed_from_wheel_speed_median(data, output_name, spec, derive_spec)

    raise ValueError(f"Unsupported derive method for {output_name}: {method}")


def derive_speed_from_wheel_speed_median(data, output_name: str, spec: dict, derive_spec: dict):
    source_specs = derive_spec.get("sources", {})
    if not source_specs:
        return None

    components = {}
    for name, source_spec in source_specs.items():
        signal = extract_direct_signal(
            data,
            f"{output_name}.{name}",
            source_spec,
            required=False,
        )
        if signal is not None:
            components[name] = signal

    minimum_channels = int(derive_spec.get("minimum_channels", 2))
    if len(components) < minimum_channels:
        return None

    freq = max(max(signal["frequency_hz"] for signal in components.values()), 1.0)
    duration = min(float(signal["time"][-1]) for signal in components.values())
    if duration <= 0:
        return None

    sample_count = max(2, int(math.floor(duration * freq)) + 1)
    time = np.arange(sample_count, dtype=float) / freq
    time = time[time <= duration + 1e-12]

    names = list(components.keys())
    matrix = np.column_stack([
        resample_signal(components[name], time, np.nan)
        for name in names
    ])
    valid_matrix, rejected = reject_failed_wheel_speed_channels(matrix, derive_spec, names)
    speed_matrix, yaw_signal = centerline_correct_wheel_speeds(
        data,
        output_name,
        derive_spec,
        time,
        valid_matrix,
        names,
    )

    values, enough = derive_speed_rows_from_components(
        speed_matrix,
        names,
        derive_spec,
        minimum_channels,
    )

    fallback_spec = derive_spec.get("fallback")
    fallback = None
    if fallback_spec:
        fallback = extract_direct_signal(
            data,
            f"{output_name}.fallback",
            fallback_spec,
            required=False,
        )
    if fallback is not None:
        fallback_values = resample_signal(fallback, time, np.nan)
        values[~np.isfinite(values)] = fallback_values[~np.isfinite(values)]

    if not np.any(np.isfinite(values)):
        return None

    if "clamp" in spec:
        lo, hi = spec["clamp"]
        values = np.clip(values, float(lo), float(hi))

    signal = {
        "name": "median valid wheel speed",
        "unit": "m/s",
        "frequency_hz": float(freq),
        "sample_count": int(len(time)),
        "scale_applied": 1.0,
        "offset_applied": 0.0,
        "source": "derived",
        "derive_method": "wheel_speed_median",
        "minimum_channels": minimum_channels,
        "rejected_components": rejected,
        "components": {
            name: signal_manifest(signal)
            for name, signal in components.items()
        },
        "time": time,
        "values": values,
    }
    if yaw_signal is not None:
        signal["name"] = "yaw-corrected median valid wheel speed"
        signal["derive_method"] = "wheel_speed_centerline_median"
        signal["centerline_correction"] = {
            "track_width_m": float(derive_spec.get("track_width_m")),
            "yaw_rate": signal_manifest(yaw_signal),
            "preferred_components": list(derive_spec.get("preferred_components", [])),
        }
    if fallback is not None:
        signal["fallback"] = signal_manifest(fallback)
    signal.update(signal_stats(values))
    return signal


def centerline_correct_wheel_speeds(
    data,
    output_name: str,
    derive_spec: dict,
    time: np.ndarray,
    wheel_matrix: np.ndarray,
    names: list[str],
) -> tuple[np.ndarray, dict | None]:
    if not bool(derive_spec.get("centerline_correction", False)):
        return wheel_matrix, None

    track_width = derive_spec.get("track_width_m")
    yaw_spec = derive_spec.get("yaw_rate")
    if track_width is None or not yaw_spec:
        return wheel_matrix, None

    try:
        track_width = float(track_width)
    except (TypeError, ValueError):
        return wheel_matrix, None
    if not np.isfinite(track_width) or track_width <= 0:
        return wheel_matrix, None

    yaw_signal = extract_direct_signal(
        data,
        f"{output_name}.yaw_rate",
        yaw_spec,
        required=False,
    )
    if yaw_signal is None:
        return wheel_matrix, None

    yaw_rate = resample_signal(yaw_signal, time, np.nan)
    corrected = np.asarray(wheel_matrix, dtype=float).copy()
    half_track = 0.5 * track_width
    for idx, name in enumerate(names):
        side_sign = wheel_speed_side_sign(name)
        if side_sign == 0:
            continue
        corrected[:, idx] = corrected[:, idx] + side_sign * yaw_rate * half_track
    return corrected, yaw_signal


def wheel_speed_side_sign(name: str) -> int:
    name = str(name).lower()
    if name.endswith("l") or name in {"fl", "rl", "left"}:
        return 1
    if name.endswith("r") or name in {"fr", "rr", "right"}:
        return -1
    return 0


def derive_speed_rows_from_components(
    matrix: np.ndarray,
    names: list[str],
    derive_spec: dict,
    minimum_channels: int,
) -> tuple[np.ndarray, np.ndarray]:
    values = np.full(matrix.shape[0], np.nan)
    enough = np.zeros(matrix.shape[0], dtype=bool)

    preferred = [
        str(name)
        for name in derive_spec.get("preferred_components", [])
        if str(name) in names
    ]
    if preferred:
        preferred_idx = [names.index(name) for name in preferred]
        preferred_matrix = matrix[:, preferred_idx]
        preferred_minimum = int(
            derive_spec.get(
                "preferred_minimum_channels",
                min(minimum_channels, len(preferred_idx)),
            )
        )
        values, enough = median_rows_with_minimum(
            preferred_matrix,
            max(1, preferred_minimum),
        )

    fallback_values, fallback_enough = median_rows_with_minimum(
        matrix,
        minimum_channels,
    )
    fill = ~np.isfinite(values) & np.isfinite(fallback_values)
    values[fill] = fallback_values[fill]
    enough = enough | fallback_enough
    return values, enough


def median_rows_with_minimum(matrix: np.ndarray, minimum_channels: int) -> tuple[np.ndarray, np.ndarray]:
    values = np.full(matrix.shape[0], np.nan)
    valid_count = np.sum(np.isfinite(matrix), axis=1)
    enough = valid_count >= minimum_channels
    if np.any(enough):
        values[enough] = np.nanmedian(matrix[enough, :], axis=1)
    return values, enough


def reject_failed_wheel_speed_channels(
    values: np.ndarray,
    derive_spec: dict,
    names: list[str] | None = None,
) -> tuple[np.ndarray, list[str]]:
    values = np.asarray(values, dtype=float).copy()
    min_valid_speed = float(derive_spec.get("min_valid_speed_mps", 0.5))
    stuck_zero_speed = float(derive_spec.get("stuck_zero_speed_mps", 0.25))
    if names is None:
        names = list(derive_spec.get("sources", {}).keys())
    rejected = []

    moving_sample = row_nanmax(values) > min_valid_speed
    for idx in range(values.shape[1]):
        channel = values[:, idx]
        moving_values = channel[moving_sample & np.isfinite(channel)]
        if moving_values.size == 0:
            continue
        if np.nanmax(np.abs(moving_values)) <= stuck_zero_speed:
            values[:, idx] = np.nan
            if idx < len(names):
                rejected.append(names[idx])

    moving_with_other = row_nanmax(values) > min_valid_speed
    failed_sample = moving_with_other[:, None] & np.isfinite(values) & (
        np.abs(values) <= stuck_zero_speed
    )
    values[failed_sample] = np.nan
    return values, rejected


def row_nanmax(values: np.ndarray) -> np.ndarray:
    finite = np.isfinite(values)
    if not np.any(finite):
        return np.full(values.shape[0], np.nan)
    return np.max(np.where(finite, values, -np.inf), axis=1)


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

    combined_pressure = combine_brake_pressures(front_pressure, rear_pressure, derive_spec)
    peak_combined_pressure = peak_pressure(combined_pressure)
    if peak_combined_pressure > np.finfo(float).eps:
        ratio = combined_pressure / peak_combined_pressure
    else:
        ratio = np.zeros_like(combined_pressure)

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
        "combine": derive_spec.get("combine", "sum"),
        "normalization": "peak_combined_pressure",
        "peak_combined_pressure_bar": float(peak_combined_pressure),
        "pressure_scale_applied": float(1.0 / peak_combined_pressure)
        if peak_combined_pressure > np.finfo(float).eps
        else 0.0,
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


def peak_pressure(values: np.ndarray) -> float:
    finite = np.asarray(values, dtype=float)
    finite = finite[np.isfinite(finite)]
    if len(finite) == 0:
        return 0.0
    return max(float(np.max(finite)), 0.0)


def combine_brake_pressures(front_pressure: np.ndarray, rear_pressure: np.ndarray, derive_spec: dict):
    combine = str(derive_spec.get("combine", "sum")).lower()
    if combine in {"sum", "combined"}:
        return front_pressure + rear_pressure
    if combine == "front":
        return front_pressure
    if combine == "rear":
        return rear_pressure
    if combine == "mean":
        return 0.5 * (front_pressure + rear_pressure)
    if combine == "weighted_mean":
        front_weight = float(derive_spec.get("front_weight", 0.5))
        rear_weight = float(derive_spec.get("rear_weight", 1.0 - front_weight))
        denom = max(front_weight + rear_weight, np.finfo(float).eps)
        return (front_weight * front_pressure + rear_weight * rear_pressure) / denom
    if combine == "max":
        return np.maximum(front_pressure, rear_pressure)

    raise ValueError(f"Unsupported brake pressure combine method: {combine}")


def signal_manifest(signal: dict) -> dict:
    return {
        "name": signal["name"],
        "unit": signal["unit"],
        "frequency_hz": signal["frequency_hz"],
        "sample_count": signal["sample_count"],
        "scale_applied": signal["scale_applied"],
        "offset_applied": signal["offset_applied"],
        "transform_applied": signal.get("transform_applied", ""),
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


def centered_moving_average(values: np.ndarray, sample_count: int) -> np.ndarray:
    sample_count = min(len(values), max(1, int(sample_count)))
    if sample_count % 2 == 0:
        sample_count = max(1, sample_count - 1)
    kernel = np.ones(sample_count, dtype=float)
    finite = np.isfinite(values)
    numerator = np.convolve(np.where(finite, values, 0.0), kernel, mode="same")
    denominator = np.convolve(finite.astype(float), kernel, mode="same")
    result = np.full(np.asarray(values).shape, np.nan)
    usable = denominator > 0
    result[usable] = numerator[usable] / denominator[usable]
    return result


def derive_gps_kinematics(
    table: dict[str, np.ndarray],
    config: dict | None,
) -> dict:
    """Derive local trace, speed, and body accelerations from GPS position."""
    if not config or not bool(config.get("enabled", False)):
        return {"status": "disabled", "sample_count": 0}
    time_s = np.asarray(table.get("time_s", []), dtype=float)
    latitude = np.asarray(table.get("gps_lat_deg", []), dtype=float)
    longitude = np.asarray(table.get("gps_lon_deg", []), dtype=float)
    valid = np.isfinite(time_s) & np.isfinite(latitude) & np.isfinite(longitude)
    if np.count_nonzero(valid) < 5:
        return {"status": "unavailable", "sample_count": int(np.count_nonzero(valid))}

    valid_time = time_s[valid]
    latitude = np.interp(time_s, valid_time, latitude[valid])
    longitude = np.interp(time_s, valid_time, longitude[valid])
    origin = int(np.flatnonzero(valid)[0])
    earth_radius_m = 6371008.8
    latitude_origin_rad = np.deg2rad(latitude[origin])
    east_m = (
        earth_radius_m
        * np.deg2rad(longitude - longitude[origin])
        * np.cos(latitude_origin_rad)
    )
    north_m = earth_radius_m * np.deg2rad(latitude - latitude[origin])

    positive_dt = np.diff(time_s)
    positive_dt = positive_dt[np.isfinite(positive_dt) & (positive_dt > 0)]
    if positive_dt.size == 0:
        return {"status": "unavailable", "sample_count": 0}
    smoothing_s = float(config.get("smoothing_window_s", 0.35))
    sample_count = max(3, int(round(smoothing_s / np.median(positive_dt))))
    # Preserve the measured trace. Smooth its derivatives rather than the
    # position itself so endpoint windows do not bias the initial speed.
    velocity_east = centered_moving_average(
        np.gradient(east_m, time_s), sample_count
    )
    velocity_north = centered_moving_average(
        np.gradient(north_m, time_s), sample_count
    )
    speed_mps = np.hypot(velocity_east, velocity_north)
    acceleration_east = centered_moving_average(
        np.gradient(velocity_east, time_s), sample_count
    )
    acceleration_north = centered_moving_average(
        np.gradient(velocity_north, time_s), sample_count
    )

    minimum_speed = float(config.get("minimum_speed_mps", 1.0))
    moving = np.isfinite(speed_mps) & (speed_mps >= minimum_speed)
    long_accel = np.full(speed_mps.shape, np.nan)
    lat_accel = np.full(speed_mps.shape, np.nan)
    long_accel[moving] = (
        velocity_east[moving] * acceleration_east[moving]
        + velocity_north[moving] * acceleration_north[moving]
    ) / speed_mps[moving]
    lat_accel[moving] = (
        velocity_east[moving] * acceleration_north[moving]
        - velocity_north[moving] * acceleration_east[moving]
    ) / speed_mps[moving]

    table["x_m"] = east_m
    table["y_m"] = north_m
    table["speed_mps"] = np.maximum(speed_mps, 0.0)
    table["long_accel_g"] = long_accel / 9.80665
    table["lat_accel_g"] = lat_accel / 9.80665
    table["distance_m"] = integrate_distance(time_s, table["speed_mps"])
    return {
        "status": "applied",
        "sample_count": int(np.count_nonzero(np.isfinite(speed_mps))),
        "smoothing_window_s": smoothing_s,
        "minimum_speed_mps": minimum_speed,
        "position_frame": "local_east_north",
        "overridden_channels": [
            "x_m",
            "y_m",
            "distance_m",
            "speed_mps",
            "long_accel_g",
            "lat_accel_g",
        ],
    }


def repair_power_inconsistent_regen(
    table: dict[str, np.ndarray],
    config: dict | None,
) -> dict | None:
    """Replace only Iq samples that violate regenerative power conservation."""
    if not config or not bool(config.get("enabled", False)):
        return None

    required = (
        "time_s",
        "regen_torque_nm",
        "motor_torque_command_nm",
        "motor_torque_delivered_nm",
        "motor_rpm",
        "pack_voltage_v",
        "pack_current_a",
    )
    if any(name not in table for name in required):
        return {
            "status": "unavailable",
            "reason": "missing required torque, speed, or pack-power columns",
        }

    time_s = np.asarray(table["time_s"], dtype=float)
    delivered_nm = np.asarray(table["motor_torque_delivered_nm"], dtype=float).copy()
    # The regen channel is a negative-torque capability/limit, not evidence
    # that regen is active. The signed motor command is authoritative when
    # available; fall back to the regen channel only for missing commands.
    regen_limit_nm = np.asarray(table["regen_torque_nm"], dtype=float)
    regen_request_nm = np.asarray(
        table["motor_torque_command_nm"], dtype=float
    ).copy()
    missing_request = ~np.isfinite(regen_request_nm)
    regen_request_nm[missing_request] = regen_limit_nm[missing_request]
    motor_rpm = np.abs(np.asarray(table["motor_rpm"], dtype=float))
    pack_power_w = (
        np.asarray(table["pack_voltage_v"], dtype=float)
        * np.asarray(table["pack_current_a"], dtype=float)
    )

    pack_advance_s = float(config.get("pack_power_advance_s", 0.0))
    if pack_advance_s:
        finite_power = np.isfinite(time_s) & np.isfinite(pack_power_w)
        if np.count_nonzero(finite_power) >= 2:
            pack_power_w = np.interp(
                time_s + pack_advance_s,
                time_s[finite_power],
                pack_power_w[finite_power],
                left=pack_power_w[finite_power][0],
                right=pack_power_w[finite_power][-1],
            )

    charging_power_w = np.maximum(0.0, -pack_power_w)
    motor_omega_radps = motor_rpm * 2.0 * np.pi / 60.0
    measured_regen_power_w = (
        np.maximum(0.0, -delivered_nm) * motor_omega_radps
    )

    minimum_charging_power_w = float(
        config.get("minimum_charging_power_w", 1000.0)
    )
    minimum_motor_speed_rpm = float(config.get("minimum_motor_speed_rpm", 300.0))
    request_threshold_nm = float(config.get("regen_request_threshold_nm", 5.0))
    power_tolerance_w = float(config.get("power_tolerance_w", 500.0))
    maximum_efficiency = float(config.get("maximum_regen_efficiency", 1.0))
    if not 0.0 < maximum_efficiency <= 1.0:
        raise ValueError("maximum_regen_efficiency must be in (0, 1]")

    active = (
        np.isfinite(delivered_nm)
        & np.isfinite(regen_request_nm)
        & np.isfinite(charging_power_w)
        & np.isfinite(motor_omega_radps)
        & (regen_request_nm <= -request_threshold_nm)
        & (charging_power_w >= minimum_charging_power_w)
        & (motor_rpm >= minimum_motor_speed_rpm)
    )
    sign_contradiction = active & (delivered_nm >= 0.0)
    power_deficit = active & (
        charging_power_w
        > maximum_efficiency * measured_regen_power_w + power_tolerance_w
    )
    repair_mask = sign_contradiction | power_deficit

    minimum_torque_nm = np.full(delivered_nm.shape, np.nan, dtype=float)
    positive_speed = motor_omega_radps > np.finfo(float).eps
    minimum_torque_nm[positive_speed] = -charging_power_w[positive_speed] / (
        maximum_efficiency * motor_omega_radps[positive_speed]
    )
    max_torque_nm = float(config.get("maximum_reconstructed_torque_nm", np.inf))
    if math.isfinite(max_torque_nm):
        minimum_torque_nm = np.maximum(minimum_torque_nm, -abs(max_torque_nm))

    repaired_nm = delivered_nm.copy()
    repaired_nm[repair_mask] = np.minimum(
        delivered_nm[repair_mask],
        minimum_torque_nm[repair_mask],
    )
    table["motor_torque_delivered_nm"] = repaired_nm

    remaining_power_deficit = repair_mask & (
        charging_power_w
        > maximum_efficiency
        * np.maximum(0.0, -repaired_nm)
        * motor_omega_radps
        + power_tolerance_w
    )
    sample_dt_s = float(np.median(np.diff(time_s))) if len(time_s) >= 2 else 0.0
    correction_nm = repaired_nm - delivered_nm
    return {
        "status": "ok",
        "method": "charging_power_minimum_shaft_torque",
        "pack_power_advance_s": pack_advance_s,
        "maximum_regen_efficiency": maximum_efficiency,
        "power_tolerance_w": power_tolerance_w,
        "repaired_sample_count": int(np.count_nonzero(repair_mask)),
        "repaired_duration_s": float(np.count_nonzero(repair_mask) * sample_dt_s),
        "sign_contradiction_sample_count": int(np.count_nonzero(sign_contradiction)),
        "power_deficit_sample_count": int(np.count_nonzero(power_deficit)),
        "remaining_power_deficit_sample_count": int(
            np.count_nonzero(remaining_power_deficit)
        ),
        "maximum_torque_correction_nm": float(
            np.max(np.abs(correction_nm[repair_mask]))
        )
        if np.any(repair_mask)
        else 0.0,
        "corrected_minimum_torque_nm": float(np.nanmin(repaired_nm)),
        "corrected_maximum_torque_nm": float(np.nanmax(repaired_nm)),
    }


def public_laps_to_ldparser(laps: str | None) -> str | None:
    """Convert public 1-based lap/range text to ldparser's 0-based contract."""
    if laps is None:
        return None

    laps = str(laps).strip()
    if not laps:
        return None

    separator = None
    if "-" in laps:
        separator = "-"
    elif ":" in laps:
        separator = ":"

    if separator is None:
        start_lap = end_lap = int(laps)
    else:
        start_text, end_text = laps.split(separator, 1)
        start_lap = int(start_text)
        end_lap = int(end_text)

    if start_lap < 1 or end_lap < start_lap:
        raise ValueError("Lap range must be 1-based and inclusive, e.g. 1 or 4-5")

    start_idx = start_lap - 1
    end_idx = end_lap - 1
    if start_idx == end_idx:
        return str(start_idx)
    return f"{start_idx}-{end_idx}"


def write_csv(path: Path, table: dict[str, np.ndarray]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    matrix = np.column_stack([table[column] for column in REPLAY_COLUMNS])
    with path.open("w", encoding="utf-8", newline="") as handle:
        handle.write(",".join(REPLAY_COLUMNS) + "\n")
        np.savetxt(handle, matrix, delimiter=",", fmt="%.12g")


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
    parser_laps = public_laps_to_ldparser(args.laps)

    ld_data = load_ldparser().fromfile(
        str(input_file),
        laps=parser_laps,
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

    postprocessing = {}
    postprocessing["gps_kinematics"] = derive_gps_kinematics(
        table,
        channel_map.get("gps_kinematics"),
    )
    regen_repair = repair_power_inconsistent_regen(
        table,
        channel_map.get("delivered_regen_power_repair"),
    )
    if regen_repair is not None:
        postprocessing["delivered_regen_power_repair"] = regen_repair

    write_csv(output_file, table)

    manifest = {
        "input_file": str(input_file),
        "output_file": str(output_file),
        "channel_map": str(channel_map_file),
        "laps": args.laps,
        "ldparser_laps": parser_laps,
        "ldx_file": args.ldx,
        "sample_frequency_hz": output_frequency,
        "sample_count": int(len(time_out)),
        "duration_s": float(time_out[-1]) if len(time_out) else 0.0,
        "postprocessing": postprocessing,
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
