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


def test_route_kde_peaks_on_the_data():
    rng = np.random.default_rng(0)
    lon, lat = rng.normal(-157, 0.1, 200), rng.normal(21, 0.1, 200)
    X, Y, Z = routes.route_kde(lon, lat)
    i, j = np.unravel_index(Z.argmax(), Z.shape)
    assert abs(X[i, j] + 157) < 0.1 and abs(Y[i, j] - 21) < 0.1
