import numpy as np
import pandas as pd

from ais_anomaly.cleaning import drop_jumps, mmsi_category, ships_only


def test_mmsi_category():
    ids = [338189586, 9110192, 993661234, 983103273, 111366100, 970123456, 0, 123456789, 41210231]
    assert list(mmsi_category(ids)) == [
        "ship", "base station", "aid to navigation", "craft of a ship", "SAR aircraft", "beacon", "invalid", "invalid", "invalid"]


def test_ships_only():
    df = pd.DataFrame({"MMSI": [338189586, 9110192, 993661234]})
    assert ships_only(df)["MMSI"].tolist() == [338189586]


def _track(mmsi, lats, lons, start="2020-01-07 01:00"):
    t = pd.Timestamp(start) + pd.to_timedelta(np.arange(len(lats)), unit="min")
    return pd.DataFrame({"MMSI": mmsi, "BaseDateTime": t, "LAT": lats, "LON": lons})


def test_drop_jumps_keeps_a_normal_track():
    # ~10 kn: 0.0028 deg lat per minute
    df = _track(1, 21 + 0.0028 * np.arange(20), np.full(20, -157.9))
    assert len(drop_jumps(df)) == 20


def test_drop_jumps_two_transmitters_sharing_an_mmsi():
    # positions alternate between Oahu (A) and Big Island (B), ~300 km apart, every minute
    a, b = (21.23, -157.87), (20.62, -155.04)
    lats = [a[0], b[0]] * 10
    lons = [a[1], b[1]] * 10
    kept = drop_jumps(_track(338189586, lats, lons))
    assert len(kept) == 10
    assert (kept["LAT"] == a[0]).all()


def test_drop_jumps_single_glitch_and_other_vessels_untouched():
    lats = 21 + 0.0028 * np.arange(10)
    lats[5] = 25.0  # one bad fix
    df = pd.concat([_track(1, lats, np.full(10, -157.9)), _track(2, [19.0, 21.0], [-155.0, -158.0], start="2020-01-07 00:00")])
    # vessel 2 has a jump too, but only between its own two reports; vessel 1 loses just the glitch
    kept = drop_jumps(df)
    assert kept.loc[kept["MMSI"] == 1, "LAT"].tolist() == [x for i, x in enumerate(lats) if i != 5]
    assert len(kept[kept["MMSI"] == 2]) == 1


def test_drop_jumps_threshold():
    # 30 kn between two reports one minute apart
    df = _track(1, [21.0, 21.0 + 30 * 1852 / 60 / 111195], [-157.9, -157.9])
    assert len(drop_jumps(df, max_speed_kn=50)) == 2
    assert len(drop_jumps(df, max_speed_kn=20)) == 1
