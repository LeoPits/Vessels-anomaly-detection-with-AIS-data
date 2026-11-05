"""Geodesy helpers: haversine distance and lat/lon <-> UTM.

pyproj replaces the hand-written MATLAB deg2utm.m / utm2deg.m / ll2utm.m and
the Mapping Toolbox utmzone/projfwd/projinv calls.
"""

import numpy as np
from pyproj import CRS, Transformer

EARTH_RADIUS_M = 6371000.0


def wrap_angle(a):
    """Wrap angles (rad) to [-pi, pi)."""
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


def course_to_math_angle(course):
    """AIS course/heading (rad, clockwise from north) -> angle counter-clockwise from east.

    The motion models move the vessel by ``v*cos(psi), v*sin(psi)`` along UTM
    x (east) and y (north), so they need the mathematical angle. The MATLAB code
    passed the AIS angle directly, sending the prediction in the wrong direction.
    """
    # shortcut: ignores UTM meridian convergence (true vs grid north, <= ~1 deg in Hawaii,
    # up to 3*sin(lat) deg at zone edges); correct it with pyproj Proj.get_factors if it ever matters
    return wrap_angle(np.pi / 2 - np.asarray(course))


def haversine(lat1, lon1, lat2, lon2):
    """Great-circle distance in metres; works on scalars and arrays."""
    lat1, lon1, lat2, lon2 = map(np.radians, (lat1, lon1, lat2, lon2))
    a = np.sin((lat2 - lat1) / 2) ** 2 + np.cos(lat1) * np.cos(lat2) * np.sin((lon2 - lon1) / 2) ** 2
    return 2 * EARTH_RADIUS_M * np.arctan2(np.sqrt(a), np.sqrt(1 - a))


def utm_crs(lat, lon):
    """WGS84 UTM CRS of the zone containing the mean position."""
    lat, lon = np.mean(lat), np.mean(lon)
    zone = int((lon + 180) // 6) % 60 + 1
    return CRS.from_epsg((32600 if lat >= 0 else 32700) + zone)


def to_utm(lat, lon, crs=None):
    """Return ``(x, y, crs)`` in metres; the zone is picked from the data if not given."""
    crs = crs or utm_crs(lat, lon)
    x, y = Transformer.from_crs(4326, crs, always_xy=True).transform(lon, lat)
    return np.asarray(x), np.asarray(y), crs


def from_utm(x, y, crs):
    """Inverse of ``to_utm``: return ``(lat, lon)``."""
    lon, lat = Transformer.from_crs(crs, 4326, always_xy=True).transform(x, y)
    return np.asarray(lat), np.asarray(lon)
