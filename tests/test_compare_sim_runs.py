import math
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "scripts"))

import compare_sim_runs as csr  # noqa: E402


# ---------------------------------------------------------------------------
# Header parsing helpers
# ---------------------------------------------------------------------------

def test_normalize_name_strips_non_alphanumeric_and_lowercases():
    assert csr.normalize_name("Engine RPM (rpm)") == "enginerpmrpm"
    assert csr.normalize_name("Speed_km/h") == "speedkmh"
    assert csr.normalize_name("") == ""


@pytest.mark.parametrize(
    "value,expected",
    [
        ("Engine RPM (rpm)", ("Engine RPM", "rpm")),
        ("Speed (km/h)", ("Speed", "km/h")),
        ("NoUnit", ("NoUnit", "")),
        ("  Padded (g)  ", ("Padded", "g")),
    ],
)
def test_split_header(value, expected):
    assert csr.split_header(value) == expected


def test_parse_headers_populates_normalized_fields():
    headers = csr.parse_headers(["Speed (km/h)", "Distance"])
    assert headers[0].base == "Speed"
    assert headers[0].unit == "km/h"
    assert headers[0].normalized_base == "speed"
    assert headers[1].unit == ""
    assert headers[1].normalized_base == "distance"


def test_find_column_matches_base_and_full_aliases():
    headers = csr.parse_headers(["Distance (m)", "Speed (km/h)"])
    assert csr.find_column(headers, ["speed"]) == 1
    assert csr.find_column(headers, ["Distance"]) == 0
    assert csr.find_column(headers, ["throttle"]) is None


def test_find_column_prefers_alias_order():
    headers = csr.parse_headers(["Speed mps", "Vehicle Speed Value"])
    assert csr.find_column(headers, ["Vehicle Speed Value", "Speed mps"]) == 1


# ---------------------------------------------------------------------------
# Numeric helpers
# ---------------------------------------------------------------------------

def test_parse_float_handles_blank_and_invalid():
    assert csr.parse_float("3.5") == 3.5
    assert math.isnan(csr.parse_float(""))
    assert math.isnan(csr.parse_float("   "))
    assert math.isnan(csr.parse_float("abc"))


def test_unit_has_is_normalization_insensitive():
    assert csr.unit_has("km/h", "kmh")
    assert csr.unit_has("mps", "mps")
    assert not csr.unit_has("g", "kmh")


def test_speed_scale_by_unit_and_base():
    assert csr.speed_scale(csr.parse_headers(["Speed (km/h)"])[0]) == 1.0
    assert csr.speed_scale(csr.parse_headers(["Speed (mph)"])[0]) == pytest.approx(1.609344)
    assert csr.speed_scale(csr.parse_headers(["Speed (m/s)"])[0]) == 3.6
    assert csr.speed_scale(csr.parse_headers(["VX"])[0]) == 3.6
    assert csr.speed_scale(csr.parse_headers(["Speed (unknown)"])[0]) == 1.0


def test_accel_scale_converts_g_to_mps2():
    assert csr.accel_scale(csr.parse_headers(["ax (g)"])[0]) == csr.G_MPS2
    assert csr.accel_scale(csr.parse_headers(["ax (m/s2)"])[0]) == 1.0


def test_percent_scale_converts_ratio_to_percent():
    assert csr.percent_scale(csr.parse_headers(["Throttle (ratio)"])[0]) == 100.0
    assert csr.percent_scale(csr.parse_headers(["Throttle (%)"])[0]) == 1.0


def test_finite_and_all_nan():
    assert csr.finite(1.0)
    assert not csr.finite(math.nan)
    assert csr.all_nan([math.nan, math.nan])
    assert not csr.all_nan([math.nan, 1.0])


def test_read_optional_handles_missing_index_and_short_rows():
    row = ["1.0", "2.0"]
    assert csr.read_optional(row, 0, 2.0) == 2.0
    assert math.isnan(csr.read_optional(row, None, 1.0))
    assert math.isnan(csr.read_optional(row, 5, 1.0))


# ---------------------------------------------------------------------------
# Channel cleaning / derivation
# ---------------------------------------------------------------------------

def test_clean_by_distance_drops_non_increasing_and_nan_speed():
    distance = [0.0, 0.0, 1.0, math.nan, 2.0, 3.0]
    channels = {
        "speed_kmh": [10.0, 11.0, 12.0, 13.0, math.nan, 15.0],
        "ax_mps2": [1.0, 1.0, 2.0, 3.0, 4.0, 5.0],
    }
    clean_distance, clean_channels = csr.clean_by_distance(distance, channels)
    # First 0.0 kept, duplicate 0.0 dropped, nan distance dropped,
    # 2.0 dropped for nan speed, 1.0 and 3.0 kept.
    assert clean_distance == [0.0, 1.0, 3.0]
    assert clean_channels["speed_kmh"] == [10.0, 12.0, 15.0]
    assert clean_channels["ax_mps2"] == [1.0, 2.0, 5.0]


def test_clean_by_distance_preserves_none_channels():
    distance = [0.0, 1.0]
    channels = {"speed_kmh": [1.0, 2.0], "ax_mps2": None}
    _, clean_channels = csr.clean_by_distance(distance, channels)
    assert clean_channels["ax_mps2"] is None


def test_derive_ax_from_speed_uses_central_difference():
    speed_kmh = [0.0, 3.6, 7.2]  # 0, 1, 2 m/s
    time_s = [0.0, 1.0, 2.0]
    ax = csr.derive_ax_from_speed(speed_kmh, time_s)
    assert ax == pytest.approx([1.0, 1.0, 1.0])


def test_derive_ax_from_speed_requires_matching_time():
    assert csr.derive_ax_from_speed([1.0, 2.0], None) is None
    assert csr.derive_ax_from_speed([1.0, 2.0], [0.0]) is None


def test_normalize_percent_channel_scales_fractions():
    assert csr.normalize_percent_channel([0.5, 1.0]) == [50.0, 100.0]
    # Already in percent, left untouched.
    assert csr.normalize_percent_channel([50.0, 100.0]) == [50.0, 100.0]
    assert csr.normalize_percent_channel(None) is None
    assert csr.normalize_percent_channel([math.nan]) is None


def test_infer_label_from_motec_filename():
    assert csr.infer_label(Path("motec_r25_autocross_20240101_120000.csv")) == "autocross"
    assert csr.infer_label(Path("plain_name.csv")) == "plain_name"


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def test_cumulative_arc_length_open_and_closed():
    points = [(0.0, 0.0), (3.0, 4.0), (6.0, 8.0)]
    assert csr.cumulative_arc_length(points, False) == [0.0, 5.0, 10.0]
    closed = csr.cumulative_arc_length(points, True)
    assert closed[-1] == pytest.approx(10.0 + 10.0)


def test_compute_curvature_straight_line_is_zero():
    assert csr.compute_curvature([(0, 0), (1, 0), (2, 0)], False) == [0.0, 0.0, 0.0]


def test_compute_curvature_unit_circle_is_unit_reciprocal_radius():
    circle = csr.compute_curvature([(1, 0), (0, 1), (-1, 0), (0, -1)], True)
    assert all(value == pytest.approx(1.0) for value in circle)


def test_compute_curvature_needs_three_points():
    assert csr.compute_curvature([(0, 0), (1, 1)], False) == [0.0, 0.0]


# ---------------------------------------------------------------------------
# Interpolation / grid
# ---------------------------------------------------------------------------

def test_interpolate_series_interpolates_and_extrapolates():
    result = csr.interpolate_series([0, 1, 2], [0, 10, 20], [0.0, 0.5, 1.0, 2.0, 3.0])
    assert result == pytest.approx([0.0, 5.0, 10.0, 20.0, 30.0])


def test_build_grid_appends_endpoint():
    assert csr.build_grid(0.0, 1.0, 0.25) == [0.0, 0.25, 0.5, 0.75, 1.0]
    assert csr.build_grid(0.0, 1.1, 0.5) == pytest.approx([0.0, 0.5, 1.0, 1.1])


def test_build_grid_rejects_non_positive_step():
    with pytest.raises(ValueError):
        csr.build_grid(0.0, 1.0, 0.0)


# ---------------------------------------------------------------------------
# Aggregation / formatting helpers
# ---------------------------------------------------------------------------

def test_mean_and_max_finite_ignore_nan():
    assert csr.mean([1.0, 3.0, math.nan]) == 2.0
    assert csr.mean([math.nan]) is None
    assert csr.max_finite([1.0, 5.0, math.nan]) == 5.0
    assert csr.max_finite([math.nan]) is None


def test_value_text_formats_and_handles_missing():
    assert csr.value_text(1.234, "g", 2) == "1.23 g"
    assert csr.value_text(None, "g") == "n/a"
    assert csr.value_text(math.nan, "g") == "n/a"


def test_winner_text_respects_direction_and_tolerance():
    assert csr.winner_text("A", "B", 10.0, 12.0, higher_is_better=True, tolerance=0.5) == "B"
    assert csr.winner_text("A", "B", 12.0, 10.0, higher_is_better=True, tolerance=0.5) == "A"
    assert csr.winner_text("A", "B", 10.0, 10.1, higher_is_better=True, tolerance=0.5) == "Tie"
    # lower is better
    assert csr.winner_text("A", "B", 12.0, 10.0, higher_is_better=False, tolerance=0.5) == "B"
    assert csr.winner_text("A", "B", None, 10.0, higher_is_better=True, tolerance=0.5) == "n/a"


def test_delta_text_uses_signed_format():
    assert csr.delta_text(10.0, 12.0, "km/h") == "+2.00 km/h"
    assert csr.delta_text(12.0, 10.0, "km/h") == "-2.00 km/h"
    assert csr.delta_text(None, 1.0, "km/h") == "n/a"


def test_make_row_composes_report_row():
    row = csr.make_row("Top speed", "A", "B", 100.0, 105.0, "km/h", "basis")
    assert isinstance(row, csr.ReportRow)
    assert row.stronger == "B"
    assert row.a_value == "100.00 km/h"
    assert row.b_value == "105.00 km/h"
    assert row.delta == "+5.00 km/h"
    assert row.basis == "basis"


def test_mask_helpers():
    values = [1.0, 2.0, 3.0, 4.0]
    mask = [True, False, True, True]
    assert csr.mask_mean(values, mask) == pytest.approx((1.0 + 3.0 + 4.0) / 3)
    assert csr.mask_count(mask) == 3


def test_mask_distance_sums_active_segments():
    grid = [0.0, 1.0, 2.0, 3.0]
    # A segment counts when either endpoint is active, so [T, T, F, F]
    # covers segments 0-1 and 1-2.
    mask = [True, True, False, False]
    assert csr.mask_distance(grid, mask) == 2.0
    assert csr.mask_distance([0.0], [True]) == 0.0


def test_combine_optional_mean():
    assert csr.combine_optional_mean([1.0, math.nan], [3.0, 4.0], 0.0) == [2.0, 4.0]
    assert csr.combine_optional_mean(None, None, 9.0) == []
    assert csr.combine_optional_mean([math.nan], None, 7.0) == [7.0]


def test_format_distance_switches_to_km():
    assert csr.format_distance(500.0) == "500.0 m"
    assert csr.format_distance(1500.0) == "1.50 km"


# ---------------------------------------------------------------------------
# CSV IO + end-to-end comparison
# ---------------------------------------------------------------------------

def _write_run_csv(path, offset_speed=0.0):
    lines = ["Distance (m),Time (s),Speed (km/h),Ref Curvature (1/m),Throttle Pedal (%),Brake (%)"]
    for i in range(11):
        distance = float(i)
        time = float(i) * 0.1
        speed = 20.0 + offset_speed + i
        curvature = 0.0 if i < 5 else 0.05
        throttle = 100.0 if i < 5 else 0.0
        brake = 0.0 if i < 5 else 20.0
        lines.append(f"{distance},{time},{speed},{curvature},{throttle},{brake}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def test_read_telemetry_csv_parses_channels(tmp_path):
    csv_path = tmp_path / "motec_r25_autocross_20240101_120000.csv"
    _write_run_csv(csv_path)
    run = csr.read_telemetry_csv(csv_path)
    assert run.label == "autocross"
    assert run.distance_m[0] == 0.0
    assert run.distance_m[-1] == 10.0
    assert run.speed_kmh[0] == 20.0
    assert run.throttle_pct[0] == 100.0
    # ax should be derived from speed/time since no ax column present.
    assert run.ax_mps2 is not None


def test_read_telemetry_csv_missing_file_raises(tmp_path):
    with pytest.raises(FileNotFoundError):
        csr.read_telemetry_csv(tmp_path / "nope.csv")


def test_read_telemetry_csv_requires_distance_column(tmp_path):
    csv_path = tmp_path / "bad.csv"
    csv_path.write_text("Speed (km/h)\n10\n20\n", encoding="utf-8")
    with pytest.raises(ValueError):
        csr.read_telemetry_csv(csv_path)


def test_load_track_csv_roundtrip(tmp_path):
    track_csv = tmp_path / "track.csv"
    track_csv.write_text(
        "station_m,curvature_1pm\n0,0.0\n1,0.01\n2,0.02\n", encoding="utf-8"
    )
    track = csr.load_track_csv(track_csv)
    assert track.station_m == [0.0, 1.0, 2.0]
    assert track.curvature_1pm == [0.0, 0.01, 0.02]


def test_load_track_curvature_none_and_missing():
    assert csr.load_track_curvature(None) is None
    result = csr.load_track_curvature("does_not_exist.csv")
    assert result is not None
    assert result.warning is not None
    assert result.station_m == []


def test_compare_runs_produces_markdown_report(tmp_path):
    a_path = tmp_path / "motec_r25_a_20240101_120000.csv"
    b_path = tmp_path / "motec_r25_b_20240101_120000.csv"
    _write_run_csv(a_path, offset_speed=0.0)
    _write_run_csv(b_path, offset_speed=2.0)
    run_a = csr.read_telemetry_csv(a_path, "A")
    run_b = csr.read_telemetry_csv(b_path, "B")
    report = csr.compare_runs(
        run_a,
        run_b,
        track=None,
        distance_step=0.5,
        corner_curvature_threshold=0.01,
        throttle_threshold=50.0,
        brake_threshold=5.0,
    )
    assert report.startswith("# Sim Run Comparison")
    assert "| Area | Stronger | A | B | Delta (B - A) | Basis |" in report
    assert "Top speed" in report
    assert "Elapsed time" in report


def test_main_writes_report_file(tmp_path):
    a_path = tmp_path / "motec_r25_a_20240101_120000.csv"
    b_path = tmp_path / "motec_r25_b_20240101_120000.csv"
    _write_run_csv(a_path, offset_speed=0.0)
    _write_run_csv(b_path, offset_speed=2.0)
    out_path = tmp_path / "report.md"
    rc = csr.main(
        [str(a_path), str(b_path), "--distance-step", "0.5", "--output", str(out_path)]
    )
    assert rc == 0
    assert out_path.exists()
    assert out_path.read_text(encoding="utf-8").startswith("# Sim Run Comparison")


def test_build_arg_parser_defaults():
    parser = csr.build_arg_parser()
    args = parser.parse_args(["a.csv", "b.csv"])
    assert args.distance_step == 0.25
    assert args.corner_curvature_threshold == 0.01
    assert args.throttle_threshold == 50.0
    assert args.brake_threshold == 5.0
