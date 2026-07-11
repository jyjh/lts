import math
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "scripts"))

import generate_vehicle as gv  # noqa: E402


# ---------------------------------------------------------------------------
# Low-level parsing helpers
# ---------------------------------------------------------------------------

@pytest.mark.parametrize(
    "text,expected",
    [
        ("43.78 N/mm", 43.78),
        ("220NM for 5s", 220.0),
        ("0.3", 0.3),
        ("-1.5e3 x", -1500.0),
        ("N/A", None),
        ("na", None),
        ("-", None),
        ("none", None),
        ("", None),
        (None, None),
        ("no digits", None),
    ],
)
def test_parse_num(text, expected):
    assert gv.parse_num(text) == expected


@pytest.mark.parametrize(
    "text,expected",
    [
        ("16.0x7.5-10", 16.0),
        ("18X6.0-10", 18.0),
        ("no size", None),
        (None, None),
    ],
)
def test_parse_tire_diameter_in(text, expected):
    assert gv.parse_tire_diameter_in(text) == expected


def test_read_rows_strips_bom_and_parses(tmp_path):
    csv_path = tmp_path / "spec.csv"
    csv_path.write_text("\ufeffWheelbase,1558\nTrack,1210\n", encoding="utf-8")
    rows = gv.read_rows(csv_path)
    assert rows[0][0] == "Wheelbase"
    assert rows[0][1] == "1558"
    assert rows[1] == ["Track", "1210"]


# ---------------------------------------------------------------------------
# Row lookup helpers
# ---------------------------------------------------------------------------

def _rows():
    return [
        ["Wheelbase & Track", "Wheelbase:", "1558", "Front Track:", "1210", "Rear Track:", "1200"],
        ["Static camber", "deg", "1.0", "", "1.2"],
        ["Static camber adjustment method", "shims"],
        ["", "empty label row"],
    ]


def test_find_row_substring_and_exact():
    rows = _rows()
    # Substring match hits the first row containing "static camber".
    assert gv.find_row(rows, "static camber")[0] == "Static camber"
    # Exact match required to disambiguate the adjustment-method row.
    assert gv.find_row(rows, "static camber", exact=True)[0] == "Static camber"
    assert gv.find_row(rows, "does-not-exist") is None


def test_find_row_text_boolean():
    rows = _rows()
    assert gv.find_row_text(rows, "wheelbase") is True
    assert gv.find_row_text(rows, "nonexistent") is False


def test_val_after_prefers_prefix_then_substring():
    row = ["Wheelbase & Track", "Wheelbase:", "1558", "Front Track:", "1210"]
    assert gv.val_after(row, "wheelbase") == "1558"
    assert gv.val_after(row, "front track") == "1210"
    assert gv.val_after(row, "missing") is None
    assert gv.val_after([], "wheelbase") is None


def test_axle_pair_reads_first_two_numbers_from_column_c():
    front, rear = gv.axle_pair(["Wheel Rate", "N/mm", "43.78", "", "52.5"])
    assert front == 43.78
    assert rear == 52.5
    assert gv.axle_pair(None) == (None, None)
    assert gv.axle_pair(["label", "unit"]) == (None, None)


def test_axle_pair_columns_reads_fixed_columns():
    row = ["Damping", "unit", "30", "speed", "extra", "40"]
    assert gv.axle_pair_columns(row) == (30.0, 40.0)
    assert gv.axle_pair_columns(None) == (None, None)
    assert gv.axle_pair_columns(["a", "b"]) == (None, None)


# ---------------------------------------------------------------------------
# Derived-value helpers
# ---------------------------------------------------------------------------

def test_corrected_wheel_rate_none_inputs():
    assert gv.corrected_wheel_rate(None, 1.0, 1.0) == (None, None)
    # Missing roll-rate context leaves the raw value untouched.
    assert gv.corrected_wheel_rate(43.78, None, 1.2) == (43.78, None)
    assert gv.corrected_wheel_rate(43.78, 100.0, 0.0) == (43.78, None)


def test_corrected_wheel_rate_applies_decimal_slip_correction():
    track = 1.2
    raw = 5.25
    # Choose a roll rate whose implied wheel rate is ~10x the raw cell.
    implied_target = 52.5
    roll_rate = implied_target * 1000.0 * (track ** 2) / 2.0 / (180.0 / math.pi)
    value, note = gv.corrected_wheel_rate(raw, roll_rate, track)
    assert value == pytest.approx(52.5)
    assert note is not None and "corrected wheel rate" in note


def test_corrected_wheel_rate_leaves_consistent_value_alone():
    value, note = gv.corrected_wheel_rate(50.0, 30.0, 1.2)
    assert value == 50.0
    assert note is None


def test_camber_curve_from_spec():
    assert gv.camber_curve_from_spec(1.0, 20.0, 0.025, 0.025) == pytest.approx([1.5, 1.0, 0.5])
    assert gv.camber_curve_from_spec(None, 20.0, 0.025, 0.025) is None


def test_constant_curve():
    assert gv.constant_curve(3.0) == [3.0, 3.0, 3.0]
    assert gv.constant_curve(None) is None


# ---------------------------------------------------------------------------
# MATLAB formatting helpers
# ---------------------------------------------------------------------------

def test_fmt_matches_baseline_style():
    assert gv.fmt(True) == "true"
    assert gv.fmt(False) == "false"
    assert gv.fmt(5) == "5"
    assert gv.fmt(3.0) == "3"
    assert gv.fmt(3.5) == "3.5"
    assert gv.fmt(0.333333333) == "0.333333"


def test_fmt_rad_as_deg_expr():
    assert gv.fmt_rad_as_deg_expr(0) == "0"
    assert gv.fmt_rad_as_deg_expr(math.pi) == "180 * pi / 180"


def test_fmt_matlab_vector():
    assert gv.fmt_matlab_vector([1.0, 2.5, 3]) == "[1 2.5 3]"


def test_src_comment_prefers_direct_then_todo_then_default():
    spec = gv.Spec(rows=[], driver_mass=68.0)
    spec.add_direct("wheelbase", 1.558, "CSV r8: 1558 mm")
    spec.add_todo("cgHeight", 0.3, "CSV r9 derivation")
    assert gv.src_comment(spec, "wheelbase", "1.558") == "  [CSV r8: 1558 mm]"
    assert gv.src_comment(spec, "cgHeight", "0.3") == "  TODO derivable: CSV r9 derivation"
    assert gv.src_comment(spec, "unmapped", "9.9") == "  [not in spec sheet -- baseline default 9.9]"


# ---------------------------------------------------------------------------
# CLI validation helpers
# ---------------------------------------------------------------------------

def test_valid_name_accepts_identifier():
    assert gv.valid_name("R26") == "R26"
    assert gv.valid_name("my_car2") == "my_car2"


@pytest.mark.parametrize("name", ["2car", "with space", "bad-name", ""])
def test_valid_name_rejects_non_identifiers(name):
    with pytest.raises(SystemExit):
        gv.valid_name(name)


# ---------------------------------------------------------------------------
# End-to-end extract + build_matlab
# ---------------------------------------------------------------------------

def _spec_rows():
    return [
        ["Wheelbase & Track", "Wheelbase:", "1558", "Front Track:", "1210", "Rear Track:", "1210"],
        ["Center of Gravity", "CG Height:", "300"],
        ["Mass without driver", "Total:", "220"],
        ["Weight Distribution", "% Front:", "50"],
        ["Wheel Rate", "N/mm", "43.78", "", "52.5"],
        ["Motion Ratio", "", "1.0"],
        ["Ride Camber", "deg/m", "20", "", "18"],
        ["Static Camber", "deg", "1.0", "", "1.2"],
        ["Static Toe", "deg", "0.1", "", "0.1"],
    ]


def test_extract_records_direct_mappings():
    spec = gv.extract(_spec_rows(), driver_mass=68.0)
    assert spec.wheelbase == pytest.approx(1.558)
    assert spec.trackWidth == pytest.approx(1.21)
    assert spec.cgHeight == pytest.approx(0.3)
    assert spec.totalMass == pytest.approx(220.0 + 68.0)
    assert spec.staticFrontWeight == pytest.approx(0.5)
    assert spec.springFront == pytest.approx(43780.0)
    assert spec.motionRatio == pytest.approx(1.0)
    direct_paths = {path for path, _, _ in spec.direct}
    assert "wheelbase" in direct_paths
    assert "suspension.front.springRate" in direct_paths


def test_build_matlab_renders_valid_function():
    spec = gv.extract(_spec_rows(), driver_mass=68.0)
    text = gv.build_matlab("TestCar", spec)
    assert text.splitlines()[0] == "function cfg = TestCar()"
    assert "cfg = lts.vehicle.VehicleConfig();" in text
    assert "cfg.wheelbase" in text
    # Provenance comment for a mapped field is present.
    assert "CSV r8" in text


def test_main_dry_run_writes_nothing(tmp_path, capsys):
    csv_path = tmp_path / "spec.csv"
    lines = [",".join(row) for row in _spec_rows()]
    csv_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    out_path = tmp_path / "TestCar.m"
    gv.main([str(csv_path), "--name", "TestCar", "--output", str(out_path), "--dry-run"])
    captured = capsys.readouterr()
    assert "DRY RUN" in captured.out
    assert "function cfg = TestCar()" in captured.out
    assert not out_path.exists()


def test_main_writes_output_file(tmp_path):
    csv_path = tmp_path / "spec.csv"
    lines = [",".join(row) for row in _spec_rows()]
    csv_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    out_path = tmp_path / "TestCar.m"
    gv.main([str(csv_path), "--name", "TestCar", "--output", str(out_path)])
    assert out_path.exists()
    assert out_path.read_text(encoding="utf-8").startswith("function cfg = TestCar()")
    # Refuses to overwrite without --force.
    with pytest.raises(SystemExit):
        gv.main([str(csv_path), "--name", "TestCar", "--output", str(out_path)])


def test_main_missing_csv_raises(tmp_path):
    with pytest.raises(SystemExit):
        gv.main([str(tmp_path / "missing.csv"), "--name", "TestCar"])
