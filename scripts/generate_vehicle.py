#!/usr/bin/env python3
"""
generate_vehicle.py  --  FSAE EV Design Spec Sheet -> MATLAB vehicle config

Parses an FSAE (EV) Design Spec Sheet CSV (the format is stable year-to-year)
and emits a MATLAB vehicle file under src/+vehicles/<Name>.m, structured
exactly like the reference configs (vehicles.baseline / VehicleConfig).

The generated file:
  * instantiates VehicleConfig() (so every field has a sane default), then
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
            s.cgHeight = cg
            s.add_direct("cgHeight", cg,
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
        if wf is not None:
            s.springFront = wf * 1000.0          # N/mm -> N/m
            s.add_direct("suspension.front.springRate", s.springFront,
                         f"CSV r21: {wf:g} N/mm wheel rate (x1000; =springRate "
                         f"when motionRatio=1, else divide by MR^2)")
        if wr is not None:
            s.springRear = wr * 1000.0
            s.add_direct("suspension.rear.springRate", s.springRear,
                         f"CSV r21: {wr:g} N/mm wheel rate (x1000)")

    # --- Motion ratio (r26) ---
    r = find_row(rows, "motion ratio")
    if r:
        mf, mr = axle_pair(r)
        m = mf if mf is not None else mr
        if m is not None:
            s.motionRatio = m
            s.add_direct("suspension.motionRatio", m, "CSV r26")

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

    # --- Steer ratio (r39) ---
    r = find_row(rows, "steer ratio")
    if r:
        sr = parse_num(val_after(r, "steer ratio"))
        if sr is not None:
            s.steeringRatio = sr
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

    # Damping coefficients from %critical (r24/25). Needs sprung corner mass.
    r24 = find_row(rows, "jounce damping")
    r25 = find_row(rows, "rebound damping")
    pct = parse_num(r24[2]) if r24 else None
    if pct is None and r25:
        pct = parse_num(r25[2])
    if pct is not None:
        k = getattr(s, "springFront", 45000)
        m_sprung = (getattr(s, "totalMass", 256) - 4 * 25) / 4.0
        c_crit = 2 * math.sqrt(k * max(m_sprung, 1.0))
        c_est = pct / 100.0 * c_crit
        s.add_todo(
            "suspension.front.dampingCoeff / reboundCoeff", "3000 / 4500",
            f"CSV r24/r25: {pct:g}% critical damping (multi-speed curve; "
            f"front). Single-value estimate C = {pct/100:.2f}*2*sqrt(k*m_sprung) "
            f"= {c_est:.0f} N*s/m (uses guessed unsprungMass 25 kg/corner). "
            f"Same % shown for rebound (r25) -- likely a data-entry repeat.")

    # bumpStopLength from jounce travel (r20).
    r20 = find_row(rows, "suspension design travel")
    if r20:
        jf = parse_num(r20[3]) if len(r20) > 3 else None
        jr = parse_num(r20[6]) if len(r20) > 6 else None
        if jf is not None:
            s.add_todo(
                "suspension.bumpStopLength", 0.025,
                f"CSV r20: jounce travel F {jf:g} mm / R "
                f"{jr if jr is not None else '?'} mm -> bumpStopLength ~ "
                f"{jf/1000:.3f} m (front). Verify against installed stop.")

    # ARB stiffness & chassis rollStiffness from roll rate (r22).
    r22 = find_row(rows, "roll rate")
    if r22:
        rlf, rlr = axle_pair(r22)
        s.add_todo(
            "suspension.frontArb/rearArb.stiffness, chassis.rollStiffness",
            "1800/1100, 55000",
            f"CSV r22: roll rate F {rlf if rlf else '?'}/R {rlr if rlr else '?'} "
            f"N*m/deg (chassis-to-wheel). Per-element ARB.stiffness and "
            f"chassis.rollStiffness must be derived via geometry/MR; left at "
            f"baseline.")

    # camberCurve from ride camber (r27) + static camber (r30).
    r27 = find_row(rows, "ride camber")
    r30 = find_row(rows, "static camber", exact=True)
    if r27:
        cf, cr = axle_pair(r27)
        sf = parse_num(r30[2]) if r30 else None
        s.add_todo(
            "suspension.geometry.front/rear.camberCurve",
            "[0.5 0 -1.5]*pi/180 / [0.25 0 -0.8]*pi/180",
            f"CSV r27: ride camber F {cf if cf else '?'}/R {cr if cr else '?'} "
            f"deg/m; r30: static camber F {sf if sf else '?'}/R ... deg. "
            f"Build the 3-pt curve: middle = static camber, slope from ride "
            f"camber (mind the sign convention: + = top-outward).")

    # toeCurve middle from static toe (r29).
    r29 = find_row(rows, "static sum toe")
    if r29:
        tf, tr = axle_pair(r29)
        s.add_todo(
            "suspension.geometry.front/rear.toeCurve",
            "[-0.05 0 0.05]*pi/180 / [0.05 0 -0.05]*pi/180",
            f"CSV r29: static toe F {tf if tf else 0}/R {tr if tr else 0} deg "
            f"(0 = neutral). Sets the curve midpoint; ends stay at baseline.")

    # unsprungMass -- component masses are listed (r47-53) but summing them is
    # error-prone (material descriptions, not clean masses).
    if find_row_text(rows, "upright assembly") or find_row_text(rows, "hub bearings"):
        s.add_todo(
            "unsprungMass", 25,
            "CSV r47-53 list upright/hub/bearing/axle/brake components as text "
            "(no clean per-corner mass). Sum them manually if needed; baseline "
            "25 kg/corner left as-is.")

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
        ("Roll center @ 1g lateral (dynamic)", "r34"),
        ("Caster / kingpin / trail / scrub radius", "r35-36"),
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
        ("suspension.geometry.*.travelGrid / motionRatioCurve", "baseline"),
        ("suspension.{front,rear}Arb.motionRatio / leverArm / enabled",
         "0.95 / 0.26 / true"),
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
    A("    cfg = VehicleConfig();")
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
    line("cfg.maxSpeed", 80,
         src_comment(s, "maxSpeed", "80") + " [m/s] (~288 km/h)")
    line("cfg.unsprungMass", 25,
         src_comment(s, "unsprungMass", "25") + " [kg/corner]")
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
    A("        'dampingCoeff', 3000, ...        % "
      + src_comment(s, "suspension.front.dampingCoeff / reboundCoeff", "3000")
      + " [N*s/m]")
    A("        'reboundCoeff', 4500);           % [N*s/m]  (see damping TODO above)")
    A("")
    A("    cfg.suspension.rear = struct( ...")
    A(f"        'springRate', {fmt(g('springRear', 39400))}, ...         % "
      f"{src_comment(s, 'suspension.rear.springRate', '42000')} [N/m]")
    A("        'dampingCoeff', 2800, ...        % [N*s/m]  (see damping TODO above)")
    A("        'reboundCoeff', 4200);           % [N*s/m]")
    A("")
    A("    cfg.suspension.motionRatio    = %s;     %% %s"
      % (fmt(g("motionRatio", 1)),
         src_comment(s, "suspension.motionRatio", "0.95")))
    A("    cfg.suspension.bumpStopLength = 0.025;    % "
      + src_comment(s, "suspension.bumpStopLength", "0.025"))
    A("    cfg.suspension.bumpStopRate   = 200000;   % [not in spec sheet] [N/m]")
    A("    cfg.suspension.tireSpringRate = 200000;   % [not in spec sheet] [N/m]")
    A("")
    A("    % Suspension geometry: per-axle lookup tables indexed by wheel travel [m].")
    A("    cfg.suspension.geometry.front = struct( ...")
    A("        'travelGrid',       [-0.05 0 0.05], ...")
    A("        'camberCurve',      [0.5 0 -1.5] * pi / 180, ...   % "
      + src_comment(s, "suspension.geometry.front/rear.camberCurve", "baseline"))
    A("        'toeCurve',         [-0.05 0 0.05] * pi / 180, ... % "
      + src_comment(s, "suspension.geometry.front/rear.toeCurve", "baseline"))
    A("        'motionRatioCurve', [0.93 0.95 0.97], ...")
    A(f"        'rollCenterHeight', {fmt(g('rchFront', 0.030))});                        % "
      f"{src_comment(s, 'suspension.geometry.front.rollCenterHeight', '0.030')}")
    A("    cfg.suspension.geometry.rear = struct( ...")
    A("        'travelGrid',       [-0.05 0 0.05], ...")
    A("        'camberCurve',      [0.25 0 -0.8] * pi / 180, ...")
    A("        'toeCurve',         [0.05 0 -0.05] * pi / 180, ...")
    A("        'motionRatioCurve', [0.94 0.95 0.96], ...")
    A(f"        'rollCenterHeight', {fmt(g('rchRear', 0.045))});                        % "
      f"{src_comment(s, 'suspension.geometry.rear.rollCenterHeight', '0.045')}")
    A("    cfg.suspension.geometry.steering = struct( ...")
    A(f"        'steeringRatio',      {fmt(g('steeringRatio', 4.856))}, ...")
    A("        'ackermann',          %s, ..."
      % fmt(g("ackermann", 0.8872)))
    A("        'maxWheelSteerAngle', 0.6, ...                      % [not in spec sheet] [rad] (~34 deg)")
    A("        'rearSteerRatio',     0.0);")
    A("")
    A("    % Anti-roll bars (baseline -- roll-rate TODO above).")
    A("    cfg.suspension.frontArb = struct( ...")
    A("        'stiffness', 1800, ...           % [N/m] at bar end")
    A("        'motionRatio', 0.95, ...         % [not in spec sheet]")
    A("        'leverArm', 0.26, ...            % [not in spec sheet] [m]")
    A("        'enabled', true);")
    A("    cfg.suspension.rearArb = struct( ...")
    A("        'stiffness', 1100, ...           % [N/m] at bar end")
    A("        'motionRatio', 0.95, ...")
    A("        'leverArm', 0.26, ...")
    A("        'enabled', true);")
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
        A("    % TODO lsd params: VehicleManager.def supplies preload/ramp/")
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
    if s.tire_text:
        A(f"    % NOTE: CSV r14 lists '{s.tire_text.strip()}'. The default tirFile below is for an")
        A("    % 18x7.5-10 Hoosier; if the size differs, supply the matching")
        A("    % .tir file and update wheelRadius accordingly.")
    A("    cfg.tire = struct( ...")
    A("        'tirFile', '43105_18x7.5_10_R25B_7.tir', ... % [verify vs CSV r14 tire size]")
    A("        'wheelInertia', 0.5, ...         % [not in spec sheet] [kg*m^2]")
    A("        'relaxationLength', 0.30, ...    % [not in spec sheet] [m]")
    A("        'wheelRadius', 0.241935, ...     % TODO: CSV r14 size -> derive rolling radius")
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

    print("\nUNMAPPED  (present in the spec sheet, no field in VehicleConfig):\n")
    for label, ref in s.unmapped:
        print(f"  [{ref:8s}] {label}")

    print("\nMISSING  (config fields with no spec-sheet source; baseline kept):\n")
    for path, default in s.missing:
        print(f"  {path:48s} = {default}")

    print("\n" + bar)
    print("  Next steps: review the TODOs above, then reference the car with")
    print(f"    config = vehicles.{name}();   in run_simulation.m")
    print(bar)


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def find_repo_root():
    """Walk up from this script to find the dir containing src/+vehicles."""
    here = Path(__file__).resolve().parent
    for cand in [here, *here.parents]:
        if (cand / "src" / "+vehicles").is_dir():
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
                   help="Vehicle/function name (-> src/+vehicles/<Name>.m). "
                        "Must be a valid MATLAB identifier.")
    p.add_argument("--driver-mass", type=float, default=68.0,
                   help="Driver mass [kg] added to car mass for totalMass (default 68).")
    p.add_argument("--output", default=None,
                   help="Output .m path (default: src/+vehicles/<Name>.m).")
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
        out_path = root / "src" / "+vehicles" / f"{name}.m"

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
