import pandas as pd

import folium
from shapely.geometry import Polygon

from ais_anomaly.maps import add_trips, waypoint_map


def _df(rows):
    """rows: (trip, lat, lon, timestamp); row order is arbitrary."""
    return pd.DataFrame(rows, columns=["trip", "LAT", "LON", "BaseDateTime"])


def test_add_trips_one_polyline_per_trip():
    df = _df([
        (0, 21.0, -158.0, "2020-01-01 00:00"),
        (0, 21.1, -158.1, "2020-01-01 00:02"),
        (1, 20.0, -157.0, "2020-01-01 00:00"),
        (1, 20.1, -157.1, "2020-01-01 00:05"),
        (1, 20.2, -157.2, "2020-01-01 00:10"),
        (2, 30.0, -160.0, "2020-01-01 00:00"),  # single point: skipped
    ])
    m = folium.Map()
    add_trips(m, df)
    lines = [c for c in m._children.values() if isinstance(c, folium.PolyLine)]
    assert len(lines) == 2


def test_add_trips_locations_are_lat_lon_in_time_order():
    df = _df([
        (0, 21.2, -158.2, "2020-01-01 00:02"),
        (0, 21.0, -158.0, "2020-01-01 00:00"),
        (0, 21.1, -158.1, "2020-01-01 00:01"),
    ])
    m = folium.Map()
    add_trips(m, df)
    line = [c for c in m._children.values() if isinstance(c, folium.PolyLine)][0]
    assert line.locations == [[21.0, -158.0], [21.1, -158.1], [21.2, -158.2]]


def test_add_trips_color_dict():
    df = _df([
        (0, 21.0, -158.0, "2020-01-01 00:00"),
        (0, 21.1, -158.1, "2020-01-01 00:01"),
        (1, 20.0, -157.0, "2020-01-01 00:00"),
        (1, 20.1, -157.1, "2020-01-01 00:01"),
    ])
    m = folium.Map()
    add_trips(m, df, color={0: "green", 1: "red"})
    lines = [c for c in m._children.values() if isinstance(c, folium.PolyLine)]
    assert sorted(line.options["color"] for line in lines) == ["green", "red"]


def test_waypoint_map_one_geojson_per_waypoint():
    hulls = pd.DataFrame({
        "name": ["WP_0", "WP_1"],
        "hull": [
            Polygon([(-158.0, 21.0), (-157.9, 21.0), (-157.9, 21.1), (-158.0, 21.1)]),
            Polygon([(-157.0, 20.0), (-156.9, 20.0), (-156.9, 20.1), (-157.0, 20.1)]),
        ],
    })
    m = waypoint_map(hulls)
    assert isinstance(m, folium.Map)
    geojsons = [c for c in m._children.values() if isinstance(c, folium.GeoJson)]
    assert len(geojsons) == 2
