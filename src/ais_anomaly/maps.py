"""Folium maps: one GeoJson polygon per waypoint, one PolyLine per trip."""

import folium


def waypoint_map(hulls, center=(21, -158), zoom=8):
    """Folium map with one GeoJson polygon per waypoint and a popup with its name.

    hulls: DataFrame with columns "name" (str) and "hull" (shapely geometry), as returned by
    ais_anomaly.routes.cluster_waypoints. Returns the folium.Map.
    """
    m = folium.Map(center, zoom_start=zoom)
    for name, hull in zip(hulls["name"], hulls["hull"]):
        folium.GeoJson(hull).add_child(folium.Popup(name)).add_to(m)
    return m


def add_trips(m, df, trip_col="trip", color="blue", weight=2, popup_col=None):
    """Add one folium.PolyLine per trip to map m and return m.

    df has columns LAT, LON, BaseDateTime and trip_col. Points of each trip are sorted by
    BaseDateTime; trips with fewer than 2 points are skipped. Locations are [lat, lon].
    color: a color string, or a dict {trip_id: color}. popup_col: if given, the popup of each
    line shows the first value of that column in the trip (e.g. "VesselName").
    """
    for trip, g in df.groupby(trip_col, sort=False):
        g = g.sort_values("BaseDateTime")
        if len(g) < 2:
            continue
        locations = [[lat, lon] for lat, lon in zip(g["LAT"], g["LON"])]
        c = color[trip] if isinstance(color, dict) else color
        line = folium.PolyLine(locations, color=c, weight=weight)
        if popup_col is not None:
            line.add_child(folium.Popup(str(g[popup_col].iloc[0])))
        line.add_to(m)
    return m
