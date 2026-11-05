"""Single-vessel track cleaning and resampling (port of preprocess_data.m)."""

import numpy as np
import pandas as pd
from scipy.interpolate import CubicSpline

KNOTS_TO_MS = 0.514444
SOG_NOT_AVAILABLE = 102.3  # knots
ANGLE_NOT_AVAILABLE = 360  # degrees: COG 360 and heading 511 both mean "not available"
ANGLE_COLUMNS = ("cog", "heading")

NOAA_COLUMNS = {"BaseDateTime": "t", "LAT": "lat", "LON": "lon", "SOG": "sog", "COG": "cog", "Heading": "heading"}
"""Rename map from NOAA CSV columns to the AIA parquet names used here."""


def preprocess(df, freq="1s"):
    """Clean one vessel's track and resample it on a regular grid.

    Expects columns ``t, lat, lon, sog [kn], cog [deg]`` and optionally
    ``heading [deg]`` and ``speed_calc``. Returns a DataFrame indexed by time with
    sog in m/s and cog/heading in rad (still nautical: clockwise from north, in
    [0, 2*pi)), interpolated with a not-a-knot cubic spline (MATLAB
    ``retime(..., 'spline')``). The first sample is dropped, as in the original.
    With ``freq=None`` the cleaned track is returned without resampling.

    AIS "not available" codes become NaN. Rows need valid lat/lon/sog/cog;
    heading may be missing (many vessels never send it) and is interpolated
    from its own valid samples.
    """
    cols = [c for c in ("t", "lat", "lon", "sog", "speed_calc", "cog", "heading") if c in df.columns]
    out = df[cols].copy()
    out["t"] = pd.to_datetime(out["t"])
    out["sog"] = out["sog"].where(out["sog"] < SOG_NOT_AVAILABLE) * KNOTS_TO_MS
    for c in ANGLE_COLUMNS:
        if c in out:
            out[c] = np.deg2rad(out[c].where(out[c] < ANGLE_NOT_AVAILABLE))
    out = out.dropna(subset=["lat", "lon", "sog", "cog"]).sort_values("t").drop_duplicates("t").set_index("t")
    if freq is None:
        return out

    grid = pd.date_range(out.index[0].ceil(freq), out.index[-1].floor(freq), freq=freq)
    t_src = (out.index - out.index[0]).total_seconds().to_numpy()
    t_dst = (grid - out.index[0]).total_seconds().to_numpy()
    resampled = pd.DataFrame({c: _spline(t_src, out[c].to_numpy(dtype=float), t_dst, c in ANGLE_COLUMNS) for c in out.columns}, index=grid)
    resampled.index.name = "t"
    return resampled.iloc[1:]


def _spline(t_src, values, t_dst, is_angle):
    """Cubic spline through the valid samples; angles are unwrapped first so 359° -> 1° does not sweep through 180°."""
    ok = ~np.isnan(values)
    if ok.sum() < 2:
        return np.full(len(t_dst), np.nan)
    v = np.unwrap(values[ok]) if is_angle else values[ok]
    y = CubicSpline(t_src[ok], v)(t_dst)
    return np.mod(y, 2 * np.pi) if is_angle else y


def sample_time(t):
    """Most common time step in seconds (MATLAB ``mode(diff(Tsec))``)."""
    return (pd.Series(pd.to_datetime(t)).diff().dropna() / pd.Timedelta("1s")).mode().iloc[0]
