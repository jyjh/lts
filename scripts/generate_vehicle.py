#!/usr/bin/env python3
"""
generate_vehicle.py  --  FSAE EV Design Spec Sheet -> MATLAB vehicle config

Parses an FSAE (EV) Design Spec Sheet CSV (the format is stable year-to-year)
and emits a MATLAB vehicle file under src/+lts/+vehicles/<Name>.m, structured
exactly like the reference configs (lts.vehicles.baseline / lts.vehicle.VehicleConfig).

The generated file:
  * instantiates lts.vehicle.VehicleConfig() (so every field has a sane default), then
  * overrides every field the spec sheet can speak to, and
  * annotates each field with its provenance:
        [CSV r8: 1558 mm]                 -> direct mapping (real value)
        TODO derivable (CSV r24 ...)      -> derivable but ambiguous (left at
                                             default, formula sketched in comment)
        [not in spec sheet]               -> no CSV source (baseline default)

It also prints a sourcing report to stdout:
    DIRECT MAPPINGS APPLIED / DERIVED (left as TODO) /
    UNMAPPED (in CSV, no config field) / MISSING (config field, not in CSV)

Usage:
    python scripts/generate_vehicle.py <spec_sheet.csv> [--name R26]
            [--driver-mass 68] [--output PATH] [--force] [--dry-run]

Only the Python standard library is used.
"""

import argparse
import csv
import math
import re
import sys
from pathlib import Path


# ---------------------------------------------------------------------------
# CSV reading & low-level helpers
# ---------------------------------------------------------------------------

def read_rows(csv_path):
    """Read the spec sheet into a list of rows (each a list of str).

    utf-8-sig strips the BOM the template ships with; the csv module correctly
    handles quoted cells that contain commas and embedded newlines.
    """
    with open(csv_path, "r", encoding="utf-8-sig", newline="") as f:
        return [row for row in csv.reader(f)]


_NUM_RE = re.compile(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?")


def parse_num(text):
    """Extract the first number from a cell as a float, or None if absent.

    Tolerates surrounding text/units ("43.78 N/mm", "220NM for 5s", "0.3"),
    and treats blank / "N/A" / "-" as missing.
    """
    if text is None:
        return None
    s = str(text).strip()
    if s == "" or s.lower() in ("n/a", "na", "-", "none"):
        return None
    m = _NUM_RE.search(s)
    if not m:
        return None
    try:
        return float(m.group(0))
    except ValueError:
        return None


def parse_tire_diameter_in(text):
    """Return the nominal tire diameter in inches from strings like 16.0x7.5-10."""
    if text is None:
        return None
    m = re.search(r"(\d+(?:\.\d+)?)\s*x", str(text), re.IGNORECASE)
    if not m:
        return None
    try:
        return float(m.group(1))
    except ValueError:
        return None


def find_row(rows, phrase, exact=False):
    """First row whose column A contains `phrase` (case-insensitive).

    If `exact`, column A must equal `phrase` after stripping. This disambiguates
    labels that are substrings of one another (e.g. 'Static camber' vs
    'Static camber adjustment method').
    """
    p = phrase.strip().lower()
    for row in rows:
        label = (row[0] if row else "").strip().lower()
        if not label:
            continue
        if (label == p) if exact else (p in label):
            return row
    return None


def find_row_text(rows, phrase, exact=False):
    """True if any row's column A matches `phrase` (used for 'in the CSV?' checks)."""
    return find_row(rows, phrase, exact=exact) is not None


def val_after(row, label_frag):
    """Raw string of the first non-empty cell that follows the cell labelled
    `label_frag` (case-insensitive). Used for 'Label:, value' layouts.

    Column A (index 0) is always the row label and is SKIPPED -- the sub-labels
    ('Wheelbase:', 'Front Track:', ...) live in columns B onward. Without this,
    'Wheelbase & Track' would itself match a 'wheelbase' lookup and return the
    units cell. Matching prefers cells that START with the fragment (after
    stripping) and only falls back to a plain substring match.
    """
    if not row:
        return None
    frag = label_frag.lower()
    cells = [c.strip() for c in row]

    def value_following(i):
        for j in range(i + 1, len(cells)):
            if cells[j] != "":
                return cells[j]
        return None

    # Pass 1: cells (from column B onward) starting with the fragment.
    for i in range(1, len(cells)):
        if cells[i].lower().startswith(frag):
            return value_following(i)
    # Pass 2: cells (from column B onward) containing the fragment anywhere.
    for i in range(1, len(cells)):
        if frag in cells[i].lower():
            return value_following(i)
    return None


def axle_pair(row):
    """(front, rear) numeric values for a single-value-per-axle row.

    The suspension-parameter rows place the front value in column C and the
    rear value in column F (verified against the reference car). We collect the
    first two numeric cells from column C onward, which is robust to the exact
    column as long as the front/rear ordering is preserved.
    """
    nums = []
    if row:
        for i in range(2, len(row)):
            n = parse_num(row[i])
            if n is not None:
                nums.append(n)
    front = nums[0] if len(nums) >= 1 else None
    rear = nums[1] if len(nums) >= 2 else None
    return front, rear


def axle_pair_columns(row, front_index=2, rear_index=5):
    """(front, rear) numeric values from fixed front/rear value columns.

    Some rows contain extra numeric cells between the front and rear values
    (for example damping speed columns), so "first two numbers" would read the
    front value and front speed. For those rows the template's front value is
    column C and rear value is column F.
    """
    if not row:
        return None, None
    front = parse_num(row[front_index]) if len(row) > front_index else None
    rear = parse_num(row[rear_index]) if len(row) > rear_index else None
    return front, rear


def corrected_wheel_rate(wheel_rate_n_per_mm, roll_rate_nm_per_deg, track_width_m):
    """Return wheel rate [N/mm], correcting obvious decimal-place CSV slips.

    Some exported spec sheets report wheel rate as 5.25 N/mm while the same
    sheet's roll-rate row and track width imply 52.5 N/mm. When the roll-rate
    implication is about 10x the raw wheel-rate cell, prefer the implied value
    and return a provenance note.
    """
    if wheel_rate_n_per_mm is None:
        return None, None
    if roll_rate_nm_per_deg is None or track_width_m is None or track_width_m <= 0:
        return wheel_rate_n_per_mm, None

    implied_n_per_mm = (
        roll_rate_nm_per_deg * (180.0 / math.pi) * 2.0 /
        (track_width_m ** 2) / 1000.0
    )
    ratio = implied_n_per_mm / max(wheel_rate_n_per_mm, sys.float_info.epsilon)
    if 8.0 <= ratio <= 12.0:
        note = (f"CSV r21 shows {wheel_rate_n_per_mm:g} N/mm, but CSV r22 "
                f"roll rate {roll_rate_nm_per_deg:g} N*m/deg at "
                f"{track_width_m * 1000:g} mm track implies "
                f"{implied_n_per_mm:.3g} N/mm; using corrected wheel rate")
        return implied_n_per_mm, note

    return wheel_rate_n_per_mm, None


def camber_curve_from_spec(static_deg, ride_camber_deg_per_m, rebound_m, jounce_m):
    """Three-point camber curve [deg] using positive travel as bump.

    The sheet reports rate as a positive magnitude. In the simulator, positive
    camber is top-outward, so bump camber gain is represented as a negative
    slope: camber = static - rate * wheelTravel.
    """
    if (static_deg is None or ride_camber_deg_per_m is None or
            rebound_m is None or jounce_m is None):
        return None
    return [
        static_deg + ride_camber_deg_per_m * rebound_m,
        static_deg,
        static_deg - ride_camber_deg_per_m * jounce_m,
    ]


def constant_curve(value):
    """Three-point constant curve helper for MATLAB table output."""
    if value is None:
        return None
    return [value, value, value]


# ---------------------------------------------------------------------------
# Spec extraction
# ---------------------------------------------------------------------------

class Spec:
    """Holds every value pulled out of the CSV, plus provenance strings."""

    def __init__(self, rows, driver_mass):
        self.rows = rows
        self.driver_mass = driver_mass
        # Each entry: (field_path, value, source_str)
        self.direct = []
        # (field_path, default_value, derivation_str)
        self.todo = []
        # (csv_label, csv_ref)
        self.unmapped = []
        # (field_path, default_value)
        self.missing = []

    # -- helpers to record provenance --
    def add_direct(self, path, value, source):
        self.direct.append((path, value, source))

    def add_todo(self, path, default, derivation):
        self.todo.append((path, default, derivation))

    def add_missing(self, path, default):
        self.missing.append((path, default))

    def add_unmapped(self, label, ref):
        self.unmapped.append((label, ref))


def extract(rows, driver_mass):
    """Pull all usable values out of the CSV into a Spec."""
    s = Spec(rows, driver_mass)
    # The spec sheet lists unsprung components as descriptions rather than a
    # clean per-corner mass. Keep the current R25/R26 estimate as the fallback
    # so derived suspension values remain consistent with the active configs.
    s.unsprungMass = 9.3

    # ======================================================================
    # TIER A -- direct mappings (real values, with unit conversion)
    # ======================================================================

    # --- Wheelbase & Track (r8) ---
    r = find_row(rows, "wheelbase")
    if r:
        wb = parse_num(val_after(r, "wheelbase"))
        ft = parse_num(val_after(r, "front track"))
        rt = parse_num(val_after(r, "rear track"))
        if wb is not None:
            s.wheelbase = wb / 1000.0          # mm -> m
            s.add_direct("wheelbase", s.wheelbase, f"CSV r8: {wb:g} mm")
        if ft is not None:
            s.trackWidth = ft / 1000.0          # mm -> m
            src = f"CSV r8: Front Track {ft:g} mm"
            if rt is not None and not math.isclose(ft, rt):
                src += f" (warning: rear track {rt:g} mm differs; using front)"
            s.add_direct("trackWidth", s.trackWidth, src)

    # --- CG height (r9) ---
    # The Units column says 'mm' but the value (0.3) can only be metres for a
    # real car (0.3 mm is absurd). baseline/R26 both use 0.3 m, confirming.
    r = find_row(rows, "center of gravity")
    if r:
        cg = parse_num(val_after(r, "cg height"))
        if cg is not None:
            s.cgHeight = cg / 1000
            s.add_direct("cgHeight", cg / 1000,
                         f"CSV r9: {cg:g} (unit column says 'mm' but value is "
                         f"clearly metres; kept as {cg:g} m per baseline/R26)")

    # --- Mass without driver (r10) + driver -> totalMass ---
    r = find_row(rows, "mass without driver")
    if r:
        car_mass = parse_num(val_after(r, "total"))
        if car_mass is not None:
            s.totalMass = car_mass + driver_mass
            s.add_direct("totalMass", s.totalMass,
                         f"CSV r10: {car_mass:g} kg + {driver_mass} kg driver")

    # --- Weight distribution (r11) ---
    r = find_row(rows, "weight distribution")
    if r:
        pf = parse_num(val_after(r, "% front"))
        if pf is not None:
            s.staticFrontWeight = pf / 100.0    # % -> fraction
            s.add_direct("staticFrontWeight", s.staticFrontWeight,
                         f"CSV r11: {pf:g}% front")

    # --- Wheel rate -> spring rate (r21) ---
    # CSV reports the WHEEL rate; config springRate is the spring rate, related
    # by wheelRate = springRate * motionRatio^2. With MR=1 they coincide.
    r = find_row(rows, "wheel rate")
    if r:
        wf, wr = axle_pair(r)
        roll_row = find_row(rows, "roll rate")
        rlf, rlr = axle_pair(roll_row) if roll_row else (None, None)
        track_width = getattr(s, "trackWidth", None)
        if wf is not None:
            wf_raw = wf
            wf, wheel_note = corrected_wheel_rate(wf, rlf, track_width)
            s.springFront = wf * 1000.0          # N/mm -> N/m
            if wheel_note:
                source = wheel_note
            else:
                source = (f"CSV r21: {wf_raw:g} N/mm wheel rate (x1000; =springRate "
                          f"when motionRatio=1, else divide by MR^2)")
            s.add_direct("suspension.front.springRate", s.springFront, source)
        if wr is not None:
            wr_raw = wr
            wr, wheel_note = corrected_wheel_rate(wr, rlr, track_width)
            s.springRear = wr * 1000.0
            if wheel_note:
                source = wheel_note
            else:
                source = f"CSV r21: {wr_raw:g} N/mm wheel rate (x1000)"
            s.add_direct("suspension.rear.springRate", s.springRear, source)

    # --- Motion ratio (r26) ---
    r = find_row(rows, "motion ratio")
    if r:
        mf, mr = axle_pair(r)
        m = mf if mf is not None else mr
        if m is not None:
            s.motionRatio = m
            s.add_direct("suspension.motionRatio", m, "CSV r26")

    # --- Design travel and bump stop free length (r20) ---
    r = find_row(rows, "suspension design travel")
    if r:
        jf = parse_num(r[3]) if len(r) > 3 else None
        rf = parse_num(r[4]) if len(r) > 4 else None
        jr = parse_num(r[6]) if len(r) > 6 else None
        rr = parse_num(r[7]) if len(r) > 7 else None
        if jf is not None and rf is not None:
            s.frontTravelGrid = [-rf / 1000.0, 0.0, jf / 1000.0]
        if jr is not None and rr is not None:
            s.rearTravelGrid = [-rr / 1000.0, 0.0, jr / 1000.0]
        jounce_values = [v for v in (jf, jr) if v is not None]
        if jounce_values:
            s.bumpStopLength = min(jounce_values) / 1000.0
            s.add_direct(
                "suspension.bumpStopLength", s.bumpStopLength,
                f"CSV r20: jounce travel F {jf if jf is not None else '?'} mm / "
                f"R {jr if jr is not None else '?'} mm")
        if hasattr(s, "frontTravelGrid") or hasattr(s, "rearTravelGrid"):
            s.add_direct(
                "suspension.geometry.*.travelGrid",
                "spec",
                f"CSV r20: F travel {rf if rf is not None else '?'} mm rebound / "
                f"{jf if jf is not None else '?'} mm jounce; "
                f"R travel {rr if rr is not None else '?'} mm rebound / "
                f"{jr if jr is not None else '?'} mm jounce")

    # --- Damping coefficients from % critical (r24/r25) ---
    r24 = find_row(rows, "jounce damping")
    r25 = find_row(rows, "rebound damping")
    jf_pct, jr_pct = axle_pair_columns(r24)
    rf_pct, rr_pct = axle_pair_columns(r25)
    total_mass = getattr(s, "totalMass", None)
    front_weight = getattr(s, "staticFrontWeight", None)
    kf = getattr(s, "springFront", None)
    kr = getattr(s, "springRear", None)
    unsprung = getattr(s, "unsprungMass", 9.3)
    if total_mass is not None and front_weight is not None:
        total_sprung = max(total_mass - 4 * unsprung, 1.0)
        front_sprung = max(total_sprung * front_weight / 2.0, 1.0)
        rear_sprung = max(total_sprung * (1.0 - front_weight) / 2.0, 1.0)
        if kf is not None:
            ccrit_f = 2.0 * math.sqrt(kf * front_sprung)
            if jf_pct is not None:
                s.frontDampingCoeff = jf_pct / 100.0 * ccrit_f
                s.add_direct(
                    "suspension.front.dampingCoeff", s.frontDampingCoeff,
                    f"CSV r24: {jf_pct:g}% critical jounce damping; "
                    f"Ccrit={ccrit_f:.0f} N*s/m using {unsprung:g} kg/corner unsprung")
            if rf_pct is not None:
                s.frontReboundCoeff = rf_pct / 100.0 * ccrit_f
                s.add_direct(
                    "suspension.front.reboundCoeff", s.frontReboundCoeff,
                    f"CSV r25: {rf_pct:g}% critical rebound damping; "
                    f"Ccrit={ccrit_f:.0f} N*s/m using {unsprung:g} kg/corner unsprung")
        if kr is not None:
            ccrit_r = 2.0 * math.sqrt(kr * rear_sprung)
            if jr_pct is not None:
                s.rearDampingCoeff = jr_pct / 100.0 * ccrit_r
                s.add_direct(
                    "suspension.rear.dampingCoeff", s.rearDampingCoeff,
                    f"CSV r24: {jr_pct:g}% critical jounce damping; "
                    f"Ccrit={ccrit_r:.0f} N*s/m using {unsprung:g} kg/corner unsprung")
            if rr_pct is not None:
                s.rearReboundCoeff = rr_pct / 100.0 * ccrit_r
                s.add_direct(
                    "suspension.rear.reboundCoeff", s.rearReboundCoeff,
                    f"CSV r25: {rr_pct:g}% critical rebound damping; "
                    f"Ccrit={ccrit_r:.0f} N*s/m using {unsprung:g} kg/corner unsprung")

    # --- Camber and toe curves from static values and ride camber (r27/r29/r30) ---
    r27 = find_row(rows, "ride camber")
    r29 = find_row(rows, "static toe")
    r30 = find_row(rows, "static camber", exact=True)
    cf, cr = axle_pair(r27) if r27 else (None, None)
    tf, tr = axle_pair(r29) if r29 else (None, None)
    sf, sr = axle_pair(r30) if r30 else (None, None)
    front_rebound = abs(getattr(s, "frontTravelGrid", [-0.0254, 0, 0.0254])[0])
    front_jounce = getattr(s, "frontTravelGrid", [-0.0254, 0, 0.0254])[2]
    rear_rebound = abs(getattr(s, "rearTravelGrid", [-0.0254, 0, 0.0254])[0])
    rear_jounce = getattr(s, "rearTravelGrid", [-0.0254, 0, 0.0254])[2]
    s.frontCamberCurveDeg = camber_curve_from_spec(sf, cf, front_rebound, front_jounce)
    s.rearCamberCurveDeg = camber_curve_from_spec(sr, cr, rear_rebound, rear_jounce)
    if s.frontCamberCurveDeg is not None or s.rearCamberCurveDeg is not None:
        s.add_direct(
            "suspension.geometry.*.camberCurve",
            "spec",
            f"CSV r27/r30: static camber F {sf if sf is not None else '?'} deg / "
            f"R {sr if sr is not None else '?'} deg; ride camber "
            f"F {cf if cf is not None else '?'} deg/m / "
            f"R {cr if cr is not None else '?'} deg/m")
    if tf is not None:
        s.frontToeCurveDeg = constant_curve(-tf / 2.0)
    if tr is not None:
        s.rearToeCurveDeg = constant_curve(-tr / 2.0)
    if hasattr(s, "frontToeCurveDeg") or hasattr(s, "rearToeCurveDeg"):
        s.add_direct(
            "suspension.geometry.*.toeCurve",
            "spec",
            f"CSV r29: static toe F {tf if tf is not None else '?'} deg / "
            f"R {tr if tr is not None else '?'} deg; interpreted as total axle "
            "toe, split equally per wheel with simulator outward-positive sign")

    if hasattr(s, "motionRatio"):
        s.frontMotionRatioCurve = constant_curve(s.motionRatio)
        s.rearMotionRatioCurve = constant_curve(s.motionRatio)
        s.add_direct(
            "suspension.geometry.*.motionRatioCurve",
            "spec",
            f"CSV r26: {s.motionRatio:g}:1 linear")

    # --- Roll center height (r33) ---
    r = find_row(rows, "roll center height")
    if r:
        rf, rr = axle_pair(r)
        if rf is not None:
            s.rchFront = rf / 1000.0             # mm -> m
            s.add_direct("suspension.geometry.front.rollCenterHeight",
                         s.rchFront, f"CSV r33: {rf:g} mm")
        if rr is not None:
            s.rchRear = rr / 1000.0
            s.add_direct("suspension.geometry.rear.rollCenterHeight",
                         s.rchRear, f"CSV r33: {rr:g} mm")

    # --- Roll center lateral position at 1g lateral acceleration (r34) ---
    r = find_row(rows, "roll center position at 1g lateral acc")
    if r:
        front_lateral = parse_num(r[4]) if len(r) > 4 else None
        rear_lateral = parse_num(r[7]) if len(r) > 7 else None
        if front_lateral is not None:
            s.rclFront = front_lateral / 1000.0
            s.add_direct("suspension.geometry.front.rollCenterLateral",
                         s.rclFront, f"CSV r34: {front_lateral:g} mm at 1g")
        if rear_lateral is not None:
            s.rclRear = rear_lateral / 1000.0
            s.add_direct("suspension.geometry.rear.rollCenterLateral",
                         s.rclRear, f"CSV r34: {rear_lateral:g} mm at 1g")

    # --- Front caster, trail, and scrub radius (r35) ---
    r = find_row(rows, "front caster, trail, and scrub radius")
    if r:
        caster = parse_num(val_after(r, "caster"))
        trail = parse_num(val_after(r, "kin trail"))
        scrub = parse_num(val_after(r, "scrub rad"))
        if caster is not None:
            s.frontCasterAngle = caster * math.pi / 180.0
            s.add_direct("suspension.geometry.front.casterAngle",
                         s.frontCasterAngle, f"CSV r35: {caster:g} deg")
        if trail is not None:
            s.frontMechanicalTrail = trail / 1000.0
            s.add_direct("suspension.geometry.front.mechanicalTrail",
                         s.frontMechanicalTrail, f"CSV r35: {trail:g} mm")
        if scrub is not None:
            s.frontScrubRadius = scrub / 1000.0
            s.add_direct("suspension.geometry.front.scrubRadius",
                         s.frontScrubRadius, f"CSV r35: {scrub:g} mm")

    # --- Front kingpin axis inclination and offset (r36) ---
    r = find_row(rows, "front kingpin axis")
    if r:
        inclination = parse_num(val_after(r, "inclination"))
        offset = parse_num(val_after(r, "offset"))
        if inclination is not None:
            s.frontKingpinInclination = inclination * math.pi / 180.0
            s.add_direct("suspension.geometry.front.kingpinInclination",
                         s.frontKingpinInclination,
                         f"CSV r36: {inclination:g} deg")
        if offset is not None:
            s.frontKingpinOffset = offset / 1000.0
            s.add_direct("suspension.geometry.front.kingpinOffset",
                         s.frontKingpinOffset, f"CSV r36: {offset:g} mm")

    # --- Steer ratio (r39) ---
    r = find_row(rows, "steer ratio")
    if r:
        sr = parse_num(val_after(r, "steer ratio"))
        if sr is not None:
            s.steeringRatio = sr
            s.steeringRatio = 1 # Manual override, due to the way the simulator works currently. TODO
            s.add_direct("suspension.geometry.steering.steeringRatio",
                         sr, "CSV r39")

    # --- Static Ackermann (r37) ---
    r = find_row(rows, "static ackermann", exact=False)
    if r:
        ak = parse_num(val_after(r, "static ackermann")) or parse_num(r[2])
        if ak is not None:
            s.ackermann = ak / 100.0             # % -> fraction
            s.add_direct("suspension.geometry.steering.ackermann",
                         s.ackermann, f"CSV r37: {ak:g}%")

    # --- Torsional stiffness (r78) -- prefer Physical Test, fall back Simulated ---
    r = find_row(rows, "torsional stiffness")
    if r:
        phys = parse_num(val_after(r, "physical test"))
        sim = parse_num(val_after(r, "simulated"))
        target = parse_num(val_after(r, "target"))
        val = phys if phys is not None else (sim if sim is not None else target)
        if val is not None:
            # N*m/deg -> N*m/rad: 1 rad = 180/pi deg, so the rad value is LARGER.
            s.torsionalRigidity = val * 180.0 / math.pi
            which = ("Physical Test" if phys is not None
                     else "Simulated" if sim is not None else "Target")
            s.add_direct("chassis.torsionalRigidity", s.torsionalRigidity,
                         f"CSV r78: {val:g} N*m/deg ({which}) x 180/pi")

    # --- Differential type (r122) ---
    r = find_row(rows, "differential type")
    diff_type = "open"
    diff_note = None
    if r:
        text = " ".join(c for c in r if c)
        low = text.lower()
        if "lsd" in low or "limited slip" in low or "drexler" in low:
            diff_type = "lsd"
            # Try to capture the ramp setting, e.g. "30deg/45deg setting".
            ramp = re.search(r"(\d+[°º]?\s*/\s*\d+[°º]?)", text)
            diff_note = ("CSV r122: Drexler M-Diff LSD"
                         + (f", ramp {ramp.group(1)}" if ramp else "")
                         + ", non-adjustable preload")
    s.diff_type = diff_type
    s.diff_note = diff_note
    if diff_type == "lsd":
        s.add_direct("powertrain.differential.type", "'lsd'",
                     diff_note or "CSV r122: LSD detected")

    # ======================================================================
    # TIER B -- derivable / ambiguous (left at baseline default, TODO comment)
    # ======================================================================

    # brakeBiasFront from line pressures + piston counts (r42-46). Rough estimate.
    r46 = find_row(rows, "force and pressures")
    r_cal = find_row(rows, "calipers")
    if r46:
        pf = parse_num(val_after(r46, "front pres"))
        pr = parse_num(val_after(r46, "rear pres"))
        if pf is not None and pr is not None:
            s.brake_pressure_front_at_1g_bar = pf
            s.brake_pressure_rear_at_1g_bar = pr
            # Crude: clamp force ~ pressure x piston-count (ignores bore diff).
            # Front 4-piston, rear 2-piston per the caliper rows.
            est = (pf * 4) / (pf * 4 + pr * 2)
            s.add_todo(
                "brakeBiasFront", 0.60,
                f"CSV r46: line pressures F {pf:g}/R {pr:g} bar, r44 pistons "
                f"F4/R2 => front clamp-fraction ~ {est:.3f} (rough; ignores "
                f"piston bore 25 vs 25.4 mm). Verify against bias bar.")

    # brakeForceCoefficient -- "@1g Deceleration" row implies the car is sized
    # for ~1g, but the config field is a tyre-limited capacity fraction.
    if find_row_text(rows, "1g deceleration"):
        s.add_todo(
            "brakeForceCoefficient", 0.70,
            "CSV r46 is a 'Force @ 1g Deceleration' table; the system is sized "
            "for ~1g but brakeForceCoefficient is tyre-limited capacity. "
            "baseline 0.70 left as-is.")

    # ARB stiffness & chassis rollStiffness from roll rate (r22).
    r22 = find_row(rows, "roll rate")
    if r22:
        rlf, rlr = axle_pair(r22)
        s.add_direct(
            "suspension.frontArb/rearArb.enabled", "false",
            f"CSV r22: roll rate F {rlf if rlf else '?'}/R {rlr if rlr else '?'} "
            "N*m/deg is already matched by the corrected wheel rates; no "
            "separate installed ARB rate is specified, so ARBs are disabled.")

    # unsprungMass -- component masses are listed (r47-53) but summing them is
    # error-prone (material descriptions, not clean masses).
    if find_row_text(rows, "upright assembly") or find_row_text(rows, "hub bearings"):
        s.add_todo(
            "unsprungMass", s.unsprungMass,
            "CSV r47-53 list upright/hub/bearing/axle/brake components as text "
            "(no clean per-corner mass). Sum them manually if needed; current "
            f"R25/R26 estimate {s.unsprungMass:g} kg/corner left as-is.")

    # ======================================================================
    # Aero whole-car totals (r131/132) -> single whole-car aero component
    # ======================================================================
    r131 = find_row(rows, "forces")
    r132 = find_row(rows, "coefficients")
    s.aero_totals = None
    if r131 and r132:
        df = parse_num(val_after(r131, "downforce"))
        drag = parse_num(val_after(r131, "drag"))
        pf_aero = parse_num(val_after(r131, "% front"))
        cl = parse_num(val_after(r132, "cl:"))
        cd = parse_num(val_after(r132, "cd:"))
        area = parse_num(val_after(r132, "ref"))
        # air density the aero was evaluated at (buried in the r131 label).
        rho_match = re.search(r"(?:ρ|rho)\s*=\s*([\d.]+)",
                              " ".join(r131), re.I)
        rho = float(rho_match.group(1)) if rho_match else None
        cla = cl * area if (cl is not None and area is not None) else None
        cda = cd * area if (cd is not None and area is not None) else None
        # Independent ClA implied by the downforce force balance at 80 kph.
        cla_force = None
        if df is not None and rho is not None:
            v = 80 / 3.6
            cla_force = df / (0.5 * rho * v * v)
        cda_force = None
        if drag is not None and rho is not None:
            v = 80 / 3.6
            cda_force = drag / (0.5 * rho * v * v)
        front_frac = None
        x_cp = None
        if pf_aero is not None:
            front_frac = pf_aero / 100.0 if pf_aero > 1.0 else pf_aero
            wheelbase = getattr(s, "wheelbase", 1.558)
            static_front = getattr(s, "staticFrontWeight", 0.50)
            x_cp = wheelbase * (front_frac - static_front)
        s.aero_totals = dict(df=df, drag=drag, pf=pf_aero, cl=cl, cd=cd,
                             area=area, cla=cla, cda=cda, cla_force=cla_force,
                             cda_force=cda_force, front_frac=front_frac,
                             x_cp=x_cp, rho=rho)
        if x_cp is not None:
            s.add_direct(
                "aero.xPosition", x_cp,
                f"CSV r131: {pf_aero:g}% front CoP -> "
                f"wheelbase*({front_frac:.4f}-staticFrontWeight)")
        if cla is not None:
            s.add_direct(
                "aero.ClA", cla,
                f"CSV r132: Cl {cl:g} * RefArea {area:g} m^2")
        elif cla_force is not None:
            s.add_direct(
                "aero.ClA", cla_force,
                f"CSV r131: downforce {df:g} N at 80 kph")
        if cda is not None:
            s.add_direct(
                "aero.CdA", cda,
                f"CSV r132: Cd {cd:g} * RefArea {area:g} m^2")
        elif cda_force is not None:
            s.add_direct(
                "aero.CdA", cda_force,
                f"CSV r131: drag {drag:g} N at 80 kph")

    # ======================================================================
    # TIER C -- present in CSV, no home in the config (reported, not emitted)
    # ======================================================================
    unmapped_specs = [
        ("Overall length / height", "r7"),
        ("% Left weight distribution", "r11"),
        ("Tire compound/make, wheel material, suspension type", "r14/18/19"),
        ("Wheel diameter/width (in)", "r15-17"),
        ("Roll camber", "r28"),
        ("Camber adjustment method", "r31"),
        ("Anti-dive / anti-squat", "r32"),
        ("Roll center height @ 1g lateral (dynamic)", "r34"),
        ("Brake rotors / master cyl / calipers / pads", "r42-45"),
        ("Upright / hub / bearing / axle hardware", "r47-53"),
        ("Ergonomics, steering wheel, instrumentation", "r55-61"),
        ("Electrical (power, loom, LV battery, telemetry)", "r64-71"),
        ("Frame construction / material / impact attenuator", "r74-82"),
        ("Tractive battery pack (energy, cells, cooling)", "r104-118"),
        ("Motor model/rpm/torque/power/controller", "r85-102"),
        ("Final drive ratio (fixed in the motor map)", "r123"),
        ("Gear speeds & half-shafts", "r124-127"),
        ("Aero configuration type & notable features", "r130/133"),
    ]
    for label, ref in unmapped_specs:
        s.add_unmapped(label, ref)

    # ======================================================================
    # TIER D -- config fields with no CSV source (kept at baseline default)
    # ======================================================================
    missing_fields = [
        ("yawInertia", 130),
        ("maxSpeed", 80),
        ("aero.pitchSensitivityClA", 0),
        ("suspension.bumpStopRate", 200000),
        ("suspension.tireSpringRate", 200000),
        ("suspension.geometry.rear steering-axis geometry", "0"),
        ("suspension.{front,rear}Arb.stiffness / motionRatio / leverArm",
         "0 / 1 / 1 (disabled; no installed ARB rate in spec)"),
        ("suspension.rollStiffnessOverride", "NaN"),
        ("suspension.coupleChassisRollToLoadTransfer", "false"),
        ("chassis.{heave,pitch,roll}Stiffness/Damping", "baseline"),
        ("powertrain.efficiency", 0.92),
        ("tire.wheelInertia / relaxationLength / "
         "rollingResistanceCoeff / bearingDragCoeff", "baseline"),
    ]
    for path, default in missing_fields:
        s.add_missing(path, default)

    # ---- tyre & motor cross-checks (informational) ----
    r14 = find_row(rows, "tire size")
    s.tire_text = None
    if r14:
        s.tire_text = (r14[2] if len(r14) > 2 and r14[2].strip() else None) \
            or (r14[5] if len(r14) > 5 else None)
    r85 = find_row(rows, "motor manufacturer")
    s.motor_text = " ".join(c for c in r85).lower() if r85 else ""

    return s


# ---------------------------------------------------------------------------
# MATLAB generation
# ---------------------------------------------------------------------------

def fmt(x):
    """Format a Python number the way baseline.m writes it."""
    if isinstance(x, bool):
        return "true" if x else "false"
    if isinstance(x, int):
        return str(x)
    if isinstance(x, float):
        if math.isfinite(x) and x == int(x) and abs(x) < 1e15:
            return str(int(x))
        return f"{x:.6g}"
    return str(x)


def fmt_rad_as_deg_expr(rad):
    """Format a radian value as a MATLAB deg*pi/180 expression."""
    if rad == 0:
        return "0"
    deg = rad * 180.0 / math.pi
    return f"{fmt(deg)} * pi / 180"


def fmt_matlab_vector(values):
    """Format a Python numeric list as a MATLAB row vector."""
    return "[" + " ".join(fmt(v) for v in values) + "]"


def src_comment(spec, path, default_fmt):
    """Build the trailing provenance comment for a field, or '' if none."""
    for p, val, src in spec.direct:
        if p == path:
            return f"  [{src}]"
    for p, default, deriv in spec.todo:
        if p == path:
            return f"  TODO derivable: {deriv}"
    # not directly mapped -> baseline default
    return f"  [not in spec sheet -- baseline default {default_fmt}]"


def build_matlab(name, s):
    """Render the vehicle file text, mirroring baseline.m section-for-section."""
    g = lambda attr, default: getattr(s, attr, default)

    lines = []
    A = lines.append

    # ---- header ---------------------------------------------------------
    A(f"function cfg = {name}()")
    A(f"    % {name} FSAE car configuration (GENERATED from the design spec sheet)")
    A("    %")
    A("    % Auto-generated by scripts/generate_vehicle.py from the FSAE EV")
    A("    % Design Spec Sheet CSV. Fields annotated with their provenance:")
    A("    %   [CSV rN: ...]            -> direct mapping (value taken from CSV)")
    A("    %   TODO derivable (...)     -> derivable but ambiguous; left at default")
    A("    %   [not in spec sheet]      -> no CSV source; baseline default kept")
    A("    %")
    A("    % Units: SI throughout (m, kg, N, s, rad, Pa).")
    A("")
    A("    cfg = lts.vehicle.VehicleConfig();")
    A("")

    # ---- vehicle-level constants ---------------------------------------
    A("    %% ====================================================================")
    A("    %  VEHICLE-LEVEL CONSTANTS")
    A("    %  Mass includes the driver. x forward, y left, z up; CG height from")
    A("    %  the ground. Pitch/roll inertia are derived in SimpleChassis from")
    A("    %  mass + geometry; only yaw inertia is specified here.")
    A("    %  ====================================================================")

    def line(assign, val, comment):
        A("    %-32s = %-12s%%%s" % (assign, fmt(val) + ";", comment))

    line("cfg.totalMass", g("totalMass", 264),
         src_comment(s, "totalMass", "256") + " (kg, with driver)")
    line("cfg.wheelbase", g("wheelbase", 1.558),
         src_comment(s, "wheelbase", "1.558") + " [m]")
    line("cfg.trackWidth", g("trackWidth", 1.21),
         src_comment(s, "trackWidth", "1.21") + " [m]")
    line("cfg.cgHeight", g("cgHeight", 0.3),
         src_comment(s, "cgHeight", "0.3") + " [m]")
    line("cfg.yawInertia", 130,
         src_comment(s, "yawInertia", "130") + " [kg*m^2]")
    line("cfg.airDensity", 1.225,
         " [kg/m^3] (ISA standard; CSV aero r131 was evaluated at rho="
         + (f"{s.aero_totals['rho']:g}" if s.aero_totals and s.aero_totals.get("rho") else "1.162")
         + ")")
    line("cfg.staticFrontWeight", g("staticFrontWeight", 0.5038),
         src_comment(s, "staticFrontWeight", "0.50") + " [0-1]")
    line("cfg.brakeBiasFront", 0.60,
         src_comment(s, "brakeBiasFront", "0.60") + " [0-1]")
    line("cfg.brakeForceCoefficient", 0.70,
         src_comment(s, "brakeForceCoefficient", "0.70"))
    brake_pressure_front = getattr(s, "brake_pressure_front_at_1g_bar", None)
    brake_pressure_rear = getattr(s, "brake_pressure_rear_at_1g_bar", None)
    if brake_pressure_front is not None and brake_pressure_rear is not None:
        A(f"    brakePressureFrontAt1gBar = {fmt(brake_pressure_front)};"
          "            %  [CSV r46: front line pressure at 1g deceleration] [bar]")
        A(f"    brakePressureRearAt1gBar = {fmt(brake_pressure_rear)};"
          "             %  [CSV r46: rear line pressure at 1g deceleration] [bar]")
        A("    cfg.brakePressure = struct( ...")
        A("        'frontForcePerBar', cfg.totalMass * 9.80665 * "
          "cfg.brakeBiasFront / brakePressureFrontAt1gBar, ...")
        A("        'rearForcePerBar', cfg.totalMass * 9.80665 * "
          "(1 - cfg.brakeBiasFront) / brakePressureRearAt1gBar);")
    line("cfg.maxSpeed", 80,
         src_comment(s, "maxSpeed", "80") + " [m/s] (~288 km/h)")
    line("cfg.unsprungMass", g("unsprungMass", 9.3),
         src_comment(s, "unsprungMass", "9.3") + " [kg/corner]")
    A("")

    # ---- aerodynamics --------------------------------------------------
    A("    %% ====================================================================")
    A("    %  AERODYNAMICS")
    A("    %  Whole-car aero is represented by one resultant at the center of")
    A("    %  pressure. xPosition > 0 is forward of CG, < 0 is behind. zPosition")
    A("    %  is kept at CG height so drag adds no artificial pitch moment.")
    A("    %  ====================================================================")
    aero_x = -0.084146
    aero_cla = 4.10
    aero_cda = 1.60
    if s.aero_totals:
        t = s.aero_totals
        aero_x = t.get("x_cp") if t.get("x_cp") is not None else aero_x
        aero_cla = (t.get("cla") if t.get("cla") is not None
                    else t.get("cla_force") if t.get("cla_force") is not None
                    else aero_cla)
        aero_cda = (t.get("cda") if t.get("cda") is not None
                    else t.get("cda_force") if t.get("cda_force") is not None
                    else aero_cda)
        A("    % NOTE: the spec sheet (r131-132) gives WHOLE-CAR aero, measured at")
        A("    % 80 kph. CSV whole-car totals:")
        bits = []
        if t.get("df") is not None:
            bits.append(f"Downforce {t['df']:.1f} N")
        if t.get("drag") is not None:
            bits.append(f"Drag {t['drag']:.1f} N")
        if t.get("pf") is not None:
            bits.append(f"%Front {t['pf']:.2f}")
        if bits:
            A("    %   " + "  ".join(bits))
        bits2 = []
        if t.get("cla") is not None:
            bits2.append(f"Cl x RefArea({t['area']:.3f} m^2) => ClA ~ {t['cla']:.2f}")
        if t.get("cda") is not None:
            bits2.append(f"CdA ~ {t['cda']:.2f}")
        if bits2:
            A("    %   " + "  ".join(bits2))
        if t.get("cla_force") is not None and t.get("cla"):
            pct = (t['cla_force'] / t['cla'] - 1) * 100
            A(f"    %   (downforce independently implies ClA ~ {t['cla_force']:.2f} "
              f"-- ~{pct:.0f}% above Cl*Area; CSV is internally inconsistent)")
        if t.get("front_frac") is not None and t.get("x_cp") is not None:
            A(f"    %   Center of pressure: {100*t['front_frac']:.2f}% front aero "
              f"=> xPosition {t['x_cp']:.3f} m from CG.")

    A("")
    A("    cfg.aero = struct( ...")
    A(f"        'xPosition', {fmt(aero_x)}, ...      % "
      f"{src_comment(s, 'aero.xPosition', '-0.084146')} [m from CG]")
    A("        'zPosition', cfg.cgHeight, ...   % kept at CG height")
    A(f"        'ClA', {fmt(aero_cla)}, ...                 % "
      f"{src_comment(s, 'aero.ClA', '4.10')} [m^2]")
    A(f"        'CdA', {fmt(aero_cda)}, ...                 % "
      f"{src_comment(s, 'aero.CdA', '1.60')} [m^2]")
    A("        'pitchSensitivityClA', 0.0);     % "
      + src_comment(s, "aero.pitchSensitivityClA", "0.0") + " [1/rad]")
    A("")

    # ---- suspension ----------------------------------------------------
    A("    %% ====================================================================")
    A("    %  SUSPENSION")
    A("    %  Per-axle spring/damper. Wheel rate = springRate * motionRatio^2.")
    A("    %  ====================================================================")
    A("")
    A("    cfg.suspension.front = struct( ...")
    A(f"        'springRate', {fmt(g('springFront', 43780))}, ...         % "
      f"{src_comment(s, 'suspension.front.springRate', '45000')} [N/m]")
    A(f"        'dampingCoeff', {fmt(g('frontDampingCoeff', 3000))}, ...        % "
      f"{src_comment(s, 'suspension.front.dampingCoeff', '3000')} [N*s/m]")
    A(f"        'reboundCoeff', {fmt(g('frontReboundCoeff', 4500))});           % "
      f"{src_comment(s, 'suspension.front.reboundCoeff', '4500')} [N*s/m]")
    A("")
    A("    cfg.suspension.rear = struct( ...")
    A(f"        'springRate', {fmt(g('springRear', 39400))}, ...         % "
      f"{src_comment(s, 'suspension.rear.springRate', '42000')} [N/m]")
    A(f"        'dampingCoeff', {fmt(g('rearDampingCoeff', 2800))}, ...        % "
      f"{src_comment(s, 'suspension.rear.dampingCoeff', '2800')} [N*s/m]")
    A(f"        'reboundCoeff', {fmt(g('rearReboundCoeff', 4200))});           % "
      f"{src_comment(s, 'suspension.rear.reboundCoeff', '4200')} [N*s/m]")
    A("")
    A("    cfg.suspension.motionRatio    = %s;     %% %s"
      % (fmt(g("motionRatio", 1)),
         src_comment(s, "suspension.motionRatio", "0.95")))
    A(f"    cfg.suspension.bumpStopLength = {fmt(g('bumpStopLength', 0.025))};    % "
      f"{src_comment(s, 'suspension.bumpStopLength', '0.025')}")
    A("    cfg.suspension.bumpStopRate   = 200000;   % [not in spec sheet] [N/m]")
    A("    cfg.suspension.tireSpringRate = 200000;   % [not in spec sheet] [N/m]")
    A("")
    A("    % Suspension geometry: per-axle lookup tables indexed by wheel travel [m].")
    front_travel = g("frontTravelGrid", [-0.05, 0, 0.05])
    rear_travel = g("rearTravelGrid", [-0.05, 0, 0.05])
    front_camber = g("frontCamberCurveDeg", [0.5, 0, -1.5])
    rear_camber = g("rearCamberCurveDeg", [0.25, 0, -0.8])
    front_toe = g("frontToeCurveDeg", [-0.05, 0, 0.05])
    rear_toe = g("rearToeCurveDeg", [0.05, 0, -0.05])
    front_mr_curve = g("frontMotionRatioCurve", [0.93, 0.95, 0.97])
    rear_mr_curve = g("rearMotionRatioCurve", [0.94, 0.95, 0.96])
    A("    cfg.suspension.geometry.front = struct( ...")
    A(f"        'travelGrid',       {fmt_matlab_vector(front_travel)}, ...       % "
      f"{src_comment(s, 'suspension.geometry.*.travelGrid', 'baseline')}")
    A(f"        'camberCurve',      {fmt_matlab_vector(front_camber)} * pi / 180, ...   % "
      f"{src_comment(s, 'suspension.geometry.*.camberCurve', 'baseline')}")
    A(f"        'toeCurve',         {fmt_matlab_vector(front_toe)} * pi / 180, ... % "
      f"{src_comment(s, 'suspension.geometry.*.toeCurve', 'baseline')}")
    A(f"        'motionRatioCurve', {fmt_matlab_vector(front_mr_curve)}, ...       % "
      f"{src_comment(s, 'suspension.geometry.*.motionRatioCurve', 'baseline')}")
    A(f"        'rollCenterHeight', {fmt(g('rchFront', 0.030))}, ...                     % "
      f"{src_comment(s, 'suspension.geometry.front.rollCenterHeight', '0.030')}")
    A(f"        'rollCenterLateral', {fmt(g('rclFront', 0))}, ...                    % "
      f"{src_comment(s, 'suspension.geometry.front.rollCenterLateral', '0')}")
    A(f"        'casterAngle',      {fmt_rad_as_deg_expr(g('frontCasterAngle', 7.0 * math.pi / 180.0))}, ...            % "
      f"{src_comment(s, 'suspension.geometry.front.casterAngle', '7.0*pi/180')}")
    A(f"        'mechanicalTrail',  {fmt(g('frontMechanicalTrail', 0.030))}, ...                     % "
      f"{src_comment(s, 'suspension.geometry.front.mechanicalTrail', '0.030')} [m]")
    A(f"        'scrubRadius',      {fmt(g('frontScrubRadius', 0.018))}, ...                     % "
      f"{src_comment(s, 'suspension.geometry.front.scrubRadius', '0.018')} [m]")
    A(f"        'kingpinInclination', {fmt_rad_as_deg_expr(g('frontKingpinInclination', 8.0 * math.pi / 180.0))}, ...       % "
      f"{src_comment(s, 'suspension.geometry.front.kingpinInclination', '8.0*pi/180')}")
    A(f"        'kingpinOffset',    {fmt(g('frontKingpinOffset', g('frontScrubRadius', 0.018)))});                        % "
      f"{src_comment(s, 'suspension.geometry.front.kingpinOffset', '0.018')} [m]")
    A("    cfg.suspension.geometry.rear = struct( ...")
    A(f"        'travelGrid',       {fmt_matlab_vector(rear_travel)}, ...")
    A(f"        'camberCurve',      {fmt_matlab_vector(rear_camber)} * pi / 180, ...")
    A(f"        'toeCurve',         {fmt_matlab_vector(rear_toe)} * pi / 180, ...")
    A(f"        'motionRatioCurve', {fmt_matlab_vector(rear_mr_curve)}, ...")
    A(f"        'rollCenterHeight', {fmt(g('rchRear', 0.045))}, ...                     % "
      f"{src_comment(s, 'suspension.geometry.rear.rollCenterHeight', '0.045')}")
    A(f"        'rollCenterLateral', {fmt(g('rclRear', 0))}, ...                    % "
      f"{src_comment(s, 'suspension.geometry.rear.rollCenterLateral', '0')}")
    A("        'casterAngle',      0, ...")
    A("        'mechanicalTrail',  0, ...")
    A("        'scrubRadius',      0, ...")
    A("        'kingpinInclination', 0, ...")
    A("        'kingpinOffset',    0);")
    A("    cfg.suspension.geometry.steering = struct( ...")
    A(f"        'steeringRatio',      {fmt(g('steeringRatio', 4.856))}, ...")
    A("        'ackermann',          %s, ..."
      % fmt(g("ackermann", 0.8872)))
    A("        'maxWheelSteerAngle', 0.6, ...                      % [not in spec sheet] [rad] (~34 deg)")
    A("        'rearSteerRatio',     0.0);")
    A("")
    A("    % Anti-roll bars: disabled unless the sheet provides an installed bar rate.")
    A("    cfg.suspension.frontArb = struct( ...")
    A("        'stiffness', 0, ...              % [not in spec sheet]")
    A("        'motionRatio', 1, ...            % [not in spec sheet]")
    A("        'leverArm', 1, ...               % [not in spec sheet] [m]")
    A("        'enabled', false);               % "
      + src_comment(s, "suspension.frontArb/rearArb.enabled", "false"))
    A("    cfg.suspension.rearArb = struct( ...")
    A("        'stiffness', 0, ...              % [not in spec sheet]")
    A("        'motionRatio', 1, ...")
    A("        'leverArm', 1, ...")
    A("        'enabled', false);")
    A("")
    A("    cfg.suspension.rollStiffnessOverride = NaN;             % [not in spec sheet] derive from springs+ARBs")
    A("    cfg.suspension.coupleChassisRollToLoadTransfer = false; % [not in spec sheet]")
    A("")

    # ---- chassis -------------------------------------------------------
    A("    %% ====================================================================")
    A("    %  CHASSIS (sprung-mass platform heave/pitch/roll)")
    A("    %  ====================================================================")
    A("    cfg.chassis = struct( ...")
    A("        'heaveStiffness', 160000, ...    % [not in spec sheet] [N/m]")
    A("        'heaveDamping', 12000, ...       % [not in spec sheet] [N*s/m]")
    A("        'pitchStiffness', 90000, ...     % [not in spec sheet] [N*m/rad]")
    A("        'pitchDamping', 6000, ...        % [not in spec sheet]")
    A("        'rollStiffness', 55000, ...      % "
      + src_comment(s, "suspension.frontArb/rearArb.stiffness, chassis.rollStiffness",
                    "55000") + " [N*m/rad]")
    A("        'rollDamping', 5000, ...         % [not in spec sheet] [N*m*s/rad]")
    tr_val = g("torsionalRigidity", 162518)
    A(f"        'torsionalRigidity', {fmt(tr_val)}, ... % "
      f"{src_comment(s, 'chassis.torsionalRigidity', '229183')} [N*m/rad]")
    A("        'torsionalDamping', 2000);       % [not in spec sheet] [N*m*s/rad]")
    A("")

    # ---- powertrain ----------------------------------------------------
    A("    %% ====================================================================")
    A("    %  POWERTRAIN")
    A("    %  Single-speed EV. matFile = '' selects the default motor map;")
    A("    %  final drive is fixed in the map. Drivetrain is RWD.")
    A("    %    differential.type: 'open' | 'locked' (spool) | 'lsd'")
    A("    %  ====================================================================")
    if "emrax 228" in s.motor_text:
        mat_note = "% matFile '' = default EMRAX 228 map (CSV r85 confirms 'Emrax 228')"
    else:
        mat_note = "% matFile '' = default map (CSV r85 motor does not match default; verify)"
    A("    " + mat_note)
    diff_c = src_comment(s, "powertrain.differential.type", "'open'")
    if s.diff_type == "lsd":
        A("    cfg.powertrain = struct( ...")
        A("        'matFile', '', ...")
        A("        'efficiency', 0.92, ...          % [not in spec sheet] [0-1]")
        A("        'differential', struct('type', 'lsd'));  % " + diff_c)
        A("    % TODO lsd params: lts.vehicle.VehicleManager.def supplies preload/ramp/")
        A("    % speedGain/biasRatio defaults if omitted; set explicitly if known.")
    else:
        A("    cfg.powertrain = struct( ...")
        A("        'matFile', '', ...")
        A("        'efficiency', 0.92, ...          % [not in spec sheet] [0-1]")
        A("        'differential', struct('type', 'open'));  % " + diff_c)
    A("")

    # ---- tire ----------------------------------------------------------
    A("    %% ====================================================================")
    A("    %  TIRE")
    A("    %  Pacejka Magic Formula (MF 6.1) via MFeval; tirFile lives in +Tire/.")
    A("    %  ====================================================================")
    tire_radius = None
    if s.tire_text:
        tire_diameter_in = parse_tire_diameter_in(s.tire_text)
        if tire_diameter_in is not None and tire_diameter_in > 0:
            tire_radius = tire_diameter_in * 0.0254 / 2.0
        A(f"    % NOTE: CSV r14 lists '{s.tire_text.strip()}'. The default tirFile below is for an")
        A("    % 18x7.5-10 Hoosier until a matching .tir is available, but")
        A("    % wheelRadius follows the nominal tire diameter from the spec sheet.")
    if tire_radius is None:
        tire_radius = 0.241935
        tire_radius_comment = "TODO: CSV r14 size -> derive rolling radius"
    else:
        tire_radius_comment = f"[CSV r14: {tire_diameter_in:g} in tire diameter / 2]"
    A("    cfg.tire = struct( ...")
    A("        'tirFile', '43105_18x7.5_10_R25B_7.tir', ... % [verify vs CSV r14 tire size]")
    A("        'wheelInertia', 0.5, ...         % [not in spec sheet] [kg*m^2]")
    A("        'relaxationLength', 0.30, ...    % [not in spec sheet] [m]")
    A(f"        'wheelRadius', {fmt(tire_radius)}, ...       % {tire_radius_comment}")
    A("        'rollingResistanceCoeff', 0.015, ... % [not in spec sheet]")
    A("        'bearingDragCoeff', 0);          % [not in spec sheet]")
    A("end")
    A("")

    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------

def print_report(s, name, out_path):
    bar = "=" * 72
    print()
    print(bar)
    print(f"  VEHICLE GENERATION REPORT  ->  {name}")
    print(f"  output: {out_path}")
    print(bar)

    print("\nDIRECT MAPPINGS APPLIED (real values written to the config):\n")
    if s.direct:
        for path, val, src in s.direct:
            print(f"  {path:48s} = {fmt(val):<12} {src}")
    else:
        print("  (none)")

    print("\nDERIVED  (in the CSV but ambiguous; left at default, TODO in file):\n")
    if s.todo:
        for path, default, deriv in s.todo:
            print(f"  {path:48s} ~ {str(default):<10} {deriv}")
    else:
        print("  (none)")

    print("\nUNMAPPED  (present in the spec sheet, no field in lts.vehicle.VehicleConfig):\n")
    for label, ref in s.unmapped:
        print(f"  [{ref:8s}] {label}")

    print("\nMISSING  (config fields with no spec-sheet source; baseline kept):\n")
    for path, default in s.missing:
        print(f"  {path:48s} = {default}")

    print("\n" + bar)
    print("  Next steps: review the TODOs above, then reference the car with")
    print(f"    config = lts.vehicles.{name}();   in src/+lts/+app/run_simulation.m")
    print(bar)


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def find_repo_root():
    """Walk up from this script to find the dir containing src/+lts/+vehicles."""
    here = Path(__file__).resolve().parent
    for cand in [here, *here.parents]:
        if (cand / "src" / "+lts" / "+vehicles").is_dir():
            return cand
    return None


def valid_name(name):
    if not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", name):
        raise SystemExit(
            f"error: --name '{name}' is not a valid MATLAB identifier "
            f"(letters, digits, underscore; must start with a letter).")
    return name


def main(argv=None):
    p = argparse.ArgumentParser(
        description="Parse an FSAE EV spec sheet CSV into a MATLAB vehicle config.")
    p.add_argument("csv", help="Path to the FSAE Design Spec Sheet CSV.")
    p.add_argument("--name", default="generated",
                   help="Vehicle/function name (-> src/+lts/+vehicles/<Name>.m). "
                        "Must be a valid MATLAB identifier.")
    p.add_argument("--driver-mass", type=float, default=68.0,
                   help="Driver mass [kg] added to car mass for totalMass (default 68).")
    p.add_argument("--output", default=None,
                   help="Output .m path (default: src/+lts/+vehicles/<Name>.m).")
    p.add_argument("--force", action="store_true",
                   help="Overwrite an existing output file.")
    p.add_argument("--dry-run", action="store_true",
                   help="Print the generated file and report; write nothing.")
    args = p.parse_args(argv)

    name = valid_name(args.name)
    csv_path = Path(args.csv)
    if not csv_path.is_file():
        raise SystemExit(f"error: CSV not found: {csv_path}")

    rows = read_rows(csv_path)
    s = extract(rows, args.driver_mass)

    matlab_text = build_matlab(name, s)

    # Resolve output path.
    if args.output:
        out_path = Path(args.output)
    else:
        root = find_repo_root() or Path.cwd()
        out_path = root / "src" / "+lts" / "+vehicles" / f"{name}.m"

    print_report(s, name, out_path)

    if args.dry_run:
        print("\n" + "#" * 72)
        print(f"# DRY RUN -- contents of {out_path}")
        print("#" * 72)
        print(matlab_text)
        return

    if out_path.exists() and not args.force:
        raise SystemExit(
            f"error: {out_path} already exists. Use --force to overwrite.")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(matlab_text, encoding="utf-8")
    print(f"\nWrote {out_path}")


if __name__ == "__main__":
    main()
