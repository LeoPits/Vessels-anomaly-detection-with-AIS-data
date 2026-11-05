"""Cleaning rules for raw NOAA AIS reports (explored in notebooks/03_data_quality.ipynb).

None of them is applied by default: notebooks and scripts call them explicitly.
"""

import numpy as np
import pandas as pd

from .geo import haversine
from .preprocessing import KNOTS_TO_MS


def mmsi_category(mmsi):
    """Classify MMSIs: 'ship', 'base station', 'aid to navigation', ... or 'invalid'.

    A ship's MMSI has 9 digits and starts with its country code (MID, first digit
    2-7). Other ranges: 00MIDxxxx base stations (7 digits once the leading zeros
    are lost as integers), 111 SAR aircraft, 970/972/974 survival beacons,
    98 craft attached to a ship, 99 aids to navigation.
    """
    s = pd.Series(np.asarray(mmsi)).astype("int64").astype(str)
    cat = pd.Series("invalid", index=s.index)
    nine = s.str.len() == 9
    cat[nine & s.str[0].isin(list("234567"))] = "ship"
    cat[(s.str.len() == 7) | (nine & s.str.startswith("00"))] = "base station"
    cat[nine & s.str.startswith("111")] = "SAR aircraft"
    cat[nine & s.str[:3].isin(["970", "972", "974"])] = "beacon"
    cat[nine & s.str.startswith("98")] = "craft of a ship"
    cat[nine & s.str.startswith("99")] = "aid to navigation"
    cat[s.isin(["0", "123456789", "111111111", "999999999"])] = "invalid"
    return cat.to_numpy()


def ships_only(ais):
    """Keep reports whose MMSI is a ship (drops stations, buoys, beacons, invalid ids)."""
    return ais[mmsi_category(ais["MMSI"]) == "ship"]


def drop_jumps(ais, max_speed_kn=50):
    """Drop reports that no ship could reach from the previous kept report of the same MMSI.

    Sequential filter: a report is kept when the distance from the last *kept*
    report is at most ``max_speed_kn * dt``. With two transmitters sharing one
    MMSI (positions alternating A, B, A, B) this keeps the A track instead of
    dropping everything, which a pairwise test would do.
    Returns the kept rows, sorted by MMSI and time.
    """
    # shortcut: the first report of each MMSI is trusted; if it is the glitch, the real track
    # after it is dropped. Upgrade to a forward/backward vote if that shows up in the data.
    df = ais.sort_values(["MMSI", "BaseDateTime"], kind="stable")
    lat, lon, mmsi = df["LAT"].to_numpy(), df["LON"].to_numpy(), df["MMSI"].to_numpy()
    t = (pd.to_datetime(df["BaseDateTime"]) - pd.Timestamp(0)) / pd.Timedelta("1s")
    t = np.asarray(t, dtype=float)
    max_ms = max_speed_kn * KNOTS_TO_MS

    # fast path: only MMSIs with at least one too-fast consecutive pair need the loop
    same = np.r_[False, mmsi[1:] == mmsi[:-1]]
    step = np.r_[0.0, haversine(lat[:-1], lon[:-1], lat[1:], lon[1:])]
    too_fast = same & (step > max_ms * np.r_[0.0, np.diff(t)])
    keep = np.ones(len(df), dtype=bool)
    for m in np.unique(mmsi[too_fast]):
        idx = np.flatnonzero(mmsi == m)
        last = idx[0]
        for i in idx[1:]:
            if haversine(lat[last], lon[last], lat[i], lon[i]) <= max_ms * (t[i] - t[last]):
                last = i
            else:
                keep[i] = False
    return df[keep]
