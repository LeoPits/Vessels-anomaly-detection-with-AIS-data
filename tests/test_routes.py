import numpy as np
import pandas as pd
from shapely import Point

from ais_anomaly import routes

A, B = (21.30, -157.87), (19.73, -155.06)  # (lat, lon) of two ports


def _voyage(mmsi, start, port_a, port_b, n_port=30, n_sea=40):
    """A vessel moored at port A, sailing to port B, moored there.

    Each vessel uses its own berth (small offset) so the ports are not single points.
    """
    off = np.array([mmsi * 1e-3, (mmsi % 2) * 1e-3])
    a, b = np.array(port_a) + off, np.array(port_b) + off
    sea = np.linspace(a, b, n_sea + 2)[1:-1]
    pos = np.vstack([np.tile(a, (n_port, 1)), sea, np.tile(b, (n_port, 1))])
    t = pd.Timestamp(start) + pd.to_timedelta(np.arange(len(pos)), unit="min")
    return pd.DataFrame({"MMSI": mmsi, "BaseDateTime": t, "LAT": pos[:, 0], "LON": pos[:, 1], "Heading": 0})


def _fleet():
    return pd.concat([_voyage(1, "2020-01-01", A, B), _voyage(2, "2020-01-02", B, A), _voyage(3, "2020-01-03", A, B)])


def test_split_by_vessel_drops_short_tracks():
    ais = pd.concat([_fleet(), _voyage(4, "2020-01-04", A, B, n_port=5, n_sea=5)])
    vessels = routes.split_by_vessel(ais, min_points=50)
    assert [v["MMSI"].iloc[0] for v in vessels] == [1, 2, 3]


def test_split_by_vessel_cuts_voyages_at_gaps():
    v = _voyage(1, "2020-01-01", A, B)
    v.loc[50:, "BaseDateTime"] += pd.Timedelta("3h")  # silence in the middle of the sea leg
    shuffled = v.sample(frac=1, random_state=0)  # input order must not matter
    assert len(routes.split_by_vessel(shuffled)) == 1
    voyages = routes.split_by_vessel(shuffled, min_points=20, max_gap="60min")
    assert [len(p) for p in voyages] == [50, 50]
    assert [p["voyage"].iloc[0] for p in voyages] == [0, 1]
    assert voyages[0]["BaseDateTime"].is_monotonic_increasing
    assert routes.split_by_vessel(shuffled, min_points=60, max_gap="60min") == []


def _with_waypoints(seq):
    t = pd.Timestamp("2020-01-01") + pd.to_timedelta(np.arange(len(seq)), unit="min")
    return pd.DataFrame({"BaseDateTime": t, "waypoints": seq})


def test_extract_routes_loops_are_optional():
    v = _with_waypoints(["WP_0", "WP_0", "-1", "-1", "-1", "WP_0", "-1", "-1", "WP_1"])
    assert [r["Route"].iloc[0] for r in routes.extract_routes(v)] == ["WP_0-WP_1"]
    found = routes.extract_routes(v, loops=True)
    assert [r["Route"].iloc[0] for r in found] == ["WP_0-WP_0", "WP_0-WP_1"]
    assert len(found[0]) == 4  # last report in WP_0 + 3 at sea


def test_speed_status():
    v = routes.add_speed_status(routes.split_by_vessel(_fleet())[0])
    assert (v["status"].iloc[1:30] == "stationary").all()
    assert (v["status"].iloc[30:71] == "sailing").all()
    leg = v["avg_speed"].iloc[31:70]  # one minute between reports
    np.testing.assert_allclose(leg, leg.iloc[0], rtol=0.01)
    assert 100 < leg.iloc[0] < 200  # ~340 km in 41 min ≈ 140 m/s


def test_pipeline_finds_two_waypoints_and_routes():
    vessels = [routes.add_speed_status(v) for v in routes.split_by_vessel(_fleet())]
    candidates = routes.waypoint_candidates(vessels)
    _, hulls = routes.cluster_waypoints(candidates, eps=0.2, min_samples=2)
    assert len(hulls) == 2

    all_routes = []
    for v in vessels:
        v["waypoints"] = routes.assign_waypoints(v, hulls)
        all_routes += routes.extract_routes(v)

    near_a = hulls["hull"].map(lambda h: h.distance(Point(A[1], A[0]))).idxmin()
    a, b = hulls["name"][near_a], hulls["name"][1 - near_a]
    assert sorted(r["Route"].iloc[0] for r in all_routes) == sorted([f"{a}-{b}", f"{b}-{a}", f"{a}-{b}"])
    assert len(all_routes[0]) == 41  # 40 reports at sea + the last one in port


def test_grid_snapping_keeps_the_same_waypoints():
    candidates = routes.waypoint_candidates([routes.add_speed_status(v) for v in routes.split_by_vessel(_fleet())])
    raw_labels, raw = routes.cluster_waypoints(candidates, eps=0.2, min_samples=2, decimals=None)
    labels, snapped = routes.cluster_waypoints(candidates, eps=0.2, min_samples=2)
    assert len(snapped) == len(raw) == 2
    np.testing.assert_array_equal(labels, raw_labels)


def test_grid_snapping_bounds_dbscan_size_on_dense_ports():
    rng = np.random.default_rng(0)
    # 60k jittered moored reports in two ports (~30 m spread) + sparse noise
    port = lambda lat, lon, n: pd.DataFrame({"LAT": rng.normal(lat, 3e-4, n), "LON": rng.normal(lon, 3e-4, n)})
    noise = pd.DataFrame({"LAT": rng.uniform(18, 22.5, 50), "LON": rng.uniform(-160, -150, 50)})
    candidates = pd.concat([port(*A, 30_000), port(*B, 30_000), noise], ignore_index=True)
    labels, hulls = routes.cluster_waypoints(candidates, eps=0.2, min_samples=20)
    assert len(labels) == len(candidates)
    assert len(hulls) == 2
    assert (labels[:60_000] != -1).all()
    assert len(np.unique(candidates[["LAT", "LON"]].round(3), axis=0)) < 200  # what DBSCAN actually sees


def test_route_kde_peaks_on_the_data():
    rng = np.random.default_rng(0)
    lon, lat = rng.normal(-157, 0.1, 200), rng.normal(21, 0.1, 200)
    X, Y, Z = routes.route_kde(lon, lat)
    i, j = np.unravel_index(Z.argmax(), Z.shape)
    assert abs(X[i, j] + 157) < 0.1 and abs(Y[i, j] - 21) < 0.1
