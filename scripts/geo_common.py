"""Shared geodesy helpers for the GPS channel extraction scripts.

Both telemetry extractors must agree on the Earth model so their derived
kinematics are mutually consistent, so the local projection lives here
instead of being re-implemented per script.
"""

import math

import numpy as np

# IUGG mean Earth radius R1 = (2a + b) / 3. The east/north projection below
# is spherical; over the few-hundred-metre spans of a lap the difference to
# an ellipsoidal projection is far below GPS noise.
EARTH_RADIUS_M = 6371008.8


def local_en_from_lat_lon(
    latitude_deg: np.ndarray,
    longitude_deg: np.ndarray,
    origin_index: int | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """Project GPS lat/lon onto a local east/north plane [m].

    The projection is anchored at ``origin_index``; when None, the first
    sample where both coordinates are finite is used.
    """
    latitude = np.asarray(latitude_deg, dtype=float)
    longitude = np.asarray(longitude_deg, dtype=float)
    if origin_index is None:
        keep = np.isfinite(latitude) & np.isfinite(longitude)
        if not np.any(keep):
            nan = np.full(latitude.shape, np.nan)
            return nan, nan.copy()
        origin_index = int(np.flatnonzero(keep)[0])

    lat0 = float(latitude[origin_index])
    lon0 = float(longitude[origin_index])
    east_m = (
        EARTH_RADIUS_M
        * np.deg2rad(longitude - lon0)
        * math.cos(math.radians(lat0))
    )
    north_m = EARTH_RADIUS_M * np.deg2rad(latitude - lat0)
    return east_m, north_m
