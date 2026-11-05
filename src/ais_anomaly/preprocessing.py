"""Single-vessel track cleaning and resampling (port of preprocess_data.m)."""

import numpy as np
import pandas as pd
from scipy.interpolate import CubicSpline

KNOTS_TO_MS = 0.514444
AIS_HEADING_NOT_AVAILABLE = 511

NOAA_COLUMNS = {"BaseDateTime": "t", "LAT": "lat", "LON": "lon", "SOG": "sog", "COG": "cog", "Heading": "heading"}
"""Rename map from NOAA CSV columns to the AIA parquet names used here."""


def preprocess(df, freq="1s"):
    """Clean one vessel's track and resample it on a regular grid.

    Expects columns ``t, lat, lon, sog [kn], cog [deg], heading [deg]`` and
    optionally ``speed_calc``. Returns a DataFrame indexed by time with sog in
    m/s and angles in rad, interpolated with a not-a-knot cubic spline
    (MATLAB ``retime(..., 'spline')``). The first sample is dropped, as in the
    original. With ``freq=None`` the cleaned track is returned without resampling.
    """
    cols = [c for c in ("t", "lat", "lon", "sog", "speed_calc", "cog", "heading") if c in df.columns]
    out = df[cols].copy()
    out["t"] = pd.to_datetime(out["t"])
    # 511 means "heading not available" in AIS; the MATLAB code treated it as degrees
    out["heading"] = out["heading"].where(out["heading"] != AIS_HEADING_NOT_AVAILABLE)
    out["sog"] = out["sog"] * KNOTS_TO_MS
    out["cog"] = np.deg2rad(out["cog"])
    out["heading"] = np.deg2rad(out["heading"])
    out = out.dropna().sort_values("t").drop_duplicates("t").set_index("t")
    if freq is None:
        return out

    grid = pd.date_range(out.index[0].ceil(freq), out.index[-1].floor(freq), freq=freq)
    t_src = (out.index - out.index[0]).total_seconds().to_numpy()
    t_dst = (grid - out.index[0]).total_seconds().to_numpy()
    resampled = pd.DataFrame({c: CubicSpline(t_src, out[c].to_numpy())(t_dst) for c in out.columns}, index=grid)
    resampled.index.name = "t"
    return resampled.iloc[1:]


def sample_time(t):
    """Most common time step in seconds (MATLAB ``mode(diff(Tsec))``)."""
    return (pd.Series(pd.to_datetime(t)).diff().dropna() / pd.Timedelta("1s")).mode().iloc[0]
