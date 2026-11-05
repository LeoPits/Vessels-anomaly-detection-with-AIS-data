"""TREAD-style route extraction: waypoints with DBSCAN, then routes between them.

Extracted from notebooks/02_route_extraction.ipynb. Each vessel is a DataFrame
with NOAA columns (MMSI, BaseDateTime, LAT, LON, ...).
"""

import numpy as np
import pandas as pd
import shapely
from scipy import stats
from shapely.geometry import MultiPoint
from sklearn.cluster import DBSCAN
from sklearn.preprocessing import StandardScaler

from .geo import haversine

NO_WAYPOINT = "-1"


def split_by_vessel(ais, min_points=50):
    """One DataFrame per MMSI, keeping vessels with at least ``min_points`` reports."""
    return [g.reset_index(drop=True) for _, g in ais.groupby("MMSI", sort=False) if len(g) >= min_points]


def add_speed_status(vessel):
    """Add ``avg_speed`` (m/s, from consecutive positions) and ``status``.

    COG/SOG are not reliable, so the speed is computed from LAT/LON; a vessel is
    'stationary' when that speed is exactly 0, 'sailing' otherwise.
    """
    v = vessel.copy()
    v["BaseDateTime"] = pd.to_datetime(v["BaseDateTime"])
    dist = haversine(v["LAT"].to_numpy()[:-1], v["LON"].to_numpy()[:-1], v["LAT"].to_numpy()[1:], v["LON"].to_numpy()[1:])
    dt = (v["BaseDateTime"].diff().iloc[1:] / pd.Timedelta("1s")).to_numpy()
    with np.errstate(divide="ignore", invalid="ignore"):
        v["avg_speed"] = np.append(0.0, dist / dt)
    v["status"] = np.where(v["avg_speed"] == 0, "stationary", "sailing")
    return v.fillna(0)


def waypoint_candidates(vessels):
    """Stationary points (POs) + first (ENs) and last (EXs) report of each vessel."""
    df = pd.concat(vessels)
    po = df[df["status"] == "stationary"]
    en = pd.DataFrame([v.iloc[0] for v in vessels])
    ex = pd.DataFrame([v.iloc[-1] for v in vessels])
    return pd.concat([po, en, ex]).drop_duplicates(subset=["LAT", "LON"], keep="first")


def cluster_waypoints(candidates, eps=0.2, min_samples=20):
    """DBSCAN on standardised LAT/LON; returns ``(labels, hulls)``.

    ``hulls`` has columns ``name`` ('WP_0', ...) and ``hull`` (convex hull of each
    cluster, lon/lat). Label -1 (noise) gets no hull. eps/min_samples are tuned by hand.
    """
    scaled = StandardScaler().fit_transform(candidates[["LAT", "LON"]])
    labels = DBSCAN(eps=eps, min_samples=min_samples).fit(scaled).labels_
    points = candidates[["LON", "LAT"]].to_numpy()
    cluster_ids = [lab for lab in pd.unique(labels) if lab != -1]
    hulls = pd.DataFrame({
        "name": [f"WP_{i}" for i in range(len(cluster_ids))],
        "hull": [MultiPoint(points[labels == lab]).convex_hull for lab in cluster_ids],
    })
    return labels, hulls


def assign_waypoints(vessel, hulls):
    """Name of the waypoint each report falls in, '-1' when none.

    A point inside several hulls gets the names concatenated, as in the notebook.
    Points on the hull boundary count as inside (the notebook's ``contains``
    excluded them, i.e. the very reports that define the hull's vertices).
    """
    lon, lat = vessel["LON"].to_numpy(), vessel["LAT"].to_numpy()
    names = np.full(len(vessel), "", dtype=object)
    for name, hull in zip(hulls["name"], hulls["hull"]):
        names[shapely.intersects_xy(hull, lon, lat)] += name
    names[names == ""] = NO_WAYPOINT
    return pd.Series(names, index=vessel.index, name="waypoints")


def extract_routes(vessel):
    """Cut a vessel track into routes between two different waypoints.

    ``vessel`` needs a ``waypoints`` column. A route is a run of '-1' reports
    between two distinct waypoints; the returned rows include the last report in
    the start waypoint and get a ``Route`` column 'WP_a-WP_b'.
    """
    wp = vessel["waypoints"].to_numpy()
    routes, i, n = [], 0, len(wp)
    while i < n:
        if wp[i] == NO_WAYPOINT:
            start, start_wp = i, max(i - 1, 0)
            while wp[i] == NO_WAYPOINT and i < n - 1:
                i += 1
            end, end_wp = i - 1, i
            if wp[start_wp] != wp[end_wp] and NO_WAYPOINT not in (wp[start_wp], wp[end_wp]):
                route = vessel.iloc[start - 1:end + 1].copy()
                route["Route"] = f"{wp[start_wp]}-{wp[end_wp]}"
                routes.append(route.reset_index())
        i += 1
    return routes


def route_kde(lon, lat, pad=0.5, n=100):
    """Gaussian KDE of positions on an ``n x n`` lon/lat grid; returns ``(X, Y, Z)``."""
    lon, lat = np.asarray(lon), np.asarray(lat)
    X, Y = np.mgrid[lon.min() - pad:lon.max() + pad:complex(n), lat.min() - pad:lat.max() + pad:complex(n)]
    kernel = stats.gaussian_kde(np.vstack([lon, lat]))
    Z = kernel(np.vstack([X.ravel(), Y.ravel()])).reshape(X.shape)
    return X, Y, Z
