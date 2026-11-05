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

from .geo import haversine, to_utm

NO_WAYPOINT = "-1"
OPEN_SEA = "SEA"  # route end where the track starts or ends at sea


def split_by_vessel(ais, min_points=50, max_gap=None):
    """One DataFrame per MMSI, sorted by time, keeping tracks with at least ``min_points`` reports.

    With ``max_gap`` (e.g. ``"60min"``) each MMSI track is also cut wherever two
    consecutive reports are more than ``max_gap`` apart. Every piece is a voyage
    with its own entry and exit point, routes never span a silence, and a
    ``voyage`` column numbers the pieces of the same MMSI. ``min_points`` then
    applies to each voyage. ``None`` (default) keeps one track per MMSI.
    """
    out = []
    for _, g in ais.groupby("MMSI", sort=False):
        g = g.sort_values("BaseDateTime", kind="stable")
        if max_gap is None:
            pieces = [g]
        else:
            new_voyage = pd.to_datetime(g["BaseDateTime"]).diff() > pd.Timedelta(max_gap)
            pieces = [p.assign(voyage=k) for k, (_, p) in enumerate(g.groupby(new_voyage.cumsum().to_numpy()))]
        out += [p.reset_index(drop=True) for p in pieces if len(p) >= min_points]
    return out


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


def cluster_waypoints(candidates, eps=0.2, min_samples=20, decimals=3):
    """DBSCAN on standardised LAT/LON; returns ``(labels, hulls)``.

    ``labels`` has one entry per candidate row (-1 = noise). ``hulls`` has columns
    ``name`` ('WP_0', ...) and ``hull`` (convex hull of each cluster, lon/lat).
    eps (in standard deviations) and min_samples are tuned by hand.

    Months of data give hundreds of thousands of candidates, and DBSCAN memory
    grows with the size of each neighbourhood. So candidates are first snapped to
    a grid of ``decimals`` decimal degrees (3 = ~100 m, far below waypoint size)
    and DBSCAN runs on the grid cells, each weighted by how many candidates it
    holds: density is counted as on the raw points. ``decimals=None`` disables it.
    """
    pts = candidates[["LAT", "LON"]].to_numpy(dtype=float)
    if decimals is not None:
        pts = pts.round(decimals)
    cells, inverse, counts = np.unique(pts, axis=0, return_inverse=True, return_counts=True)
    scaled = StandardScaler().fit_transform(cells, sample_weight=counts)  # same scaling as on all points
    cell_labels = DBSCAN(eps=eps, min_samples=min_samples).fit(scaled, sample_weight=counts).labels_
    labels = cell_labels[inverse.ravel()]

    cluster_ids = [lab for lab in pd.unique(labels) if lab != -1]  # order of first appearance, as before
    lonlat = cells[:, ::-1]
    hulls = pd.DataFrame({
        "name": [f"WP_{i}" for i in range(len(cluster_ids))],
        "hull": [MultiPoint(lonlat[cell_labels == lab]).convex_hull for lab in cluster_ids],
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


def extract_routes(vessel, loops=False, open_ends=False):
    """Cut a vessel track into routes between two different waypoints.

    ``vessel`` needs a ``waypoints`` column. A route is a run of '-1' reports
    between two distinct waypoints; the returned rows include the last report in
    the start waypoint and get a ``Route`` column 'WP_a-WP_b'.

    With ``loops=True`` a trip that leaves a waypoint and comes back to it (e.g. a
    fishing trip) is kept as route 'WP_a-WP_a' instead of being discarded.

    With ``open_ends=True`` a track that starts at sea and reaches a waypoint
    becomes route 'SEA-WP_a', and one that leaves a waypoint and ends at sea
    becomes 'WP_a-SEA'. Offshore, AIS coverage is lost long before the area's
    edge, so most fishing trips look like this. A track never inside a waypoint
    has no anchor and is still skipped.
    """
    wp = vessel["waypoints"].to_numpy()
    n = len(wp)
    at_sea = (wp == NO_WAYPOINT).astype(int)
    edges = np.flatnonzero(np.diff(np.r_[0, at_sea, 0]))  # each at-sea run is [start, stop)
    found = []
    for start, stop in zip(edges[::2], edges[1::2]):
        before = wp[start - 1] if start > 0 else OPEN_SEA
        after = wp[stop] if stop < n else OPEN_SEA
        if OPEN_SEA in (before, after):
            if not open_ends or before == after:
                continue
        elif before == after and not loops:
            continue
        route = vessel.iloc[max(start - 1, 0):stop].copy()
        route["Route"] = f"{before}-{after}"
        found.append(route.reset_index())
    return found


def score_route(route, percentile=5, max_fit=5000, seed=0):
    """Anomaly score of every report of one route, from the KDE of all its positions (TREAD).

    ``route`` holds the reports of one route (all trips), with LAT/LON. Positions
    go to UTM metres, a Gaussian KDE is fitted on them (on a random sample of
    ``max_fit`` points for large routes, to keep it fast), and each report gets:

    - ``density``: KDE value at its position (low = rarely visited by this route);
    - ``anomaly``: True when the density is below the ``percentile``-th percentile
      of the route's densities, i.e. the least travelled ``percentile`` % of positions.

    Returns a copy of ``route`` with the two columns. Note that the KDE includes
    the trip being scored, so a trip unlike all the others still raises the
    density a little where it passes.
    """
    x, y, _ = to_utm(route["LAT"].to_numpy(), route["LON"].to_numpy())
    xy = np.vstack([x, y])
    fit = xy
    if xy.shape[1] > max_fit:
        fit = xy[:, np.random.default_rng(seed).choice(xy.shape[1], max_fit, replace=False)]
    density = stats.gaussian_kde(fit)(xy)
    out = route.copy()
    out["density"] = density
    out["anomaly"] = density < np.percentile(density, percentile)
    return out


def route_kde(lon, lat, pad=0.5, n=100):
    """Gaussian KDE of positions on an ``n x n`` lon/lat grid; returns ``(X, Y, Z)``."""
    lon, lat = np.asarray(lon), np.asarray(lat)
    X, Y = np.mgrid[lon.min() - pad:lon.max() + pad:complex(n), lat.min() - pad:lat.max() + pad:complex(n)]
    kernel = stats.gaussian_kde(np.vstack([lon, lat]))
    Z = kernel(np.vstack([X.ravel(), Y.ravel()])).reshape(X.shape)
    return X, Y, Z
