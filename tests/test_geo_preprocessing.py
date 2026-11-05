import numpy as np
import pandas as pd
import pytest

from ais_anomaly.geo import from_utm, haversine, to_utm
from ais_anomaly.preprocessing import KNOTS_TO_MS, preprocess, sample_time


def test_haversine_one_degree_of_latitude():
    assert haversine(0, 0, 1, 0) == pytest.approx(111195, rel=1e-4)
    np.testing.assert_allclose(haversine([0, 0], [0, 0], [0, 1], [0, 0]), [0, 111195], rtol=1e-4)


def test_utm_round_trip_hawaii():
    lat, lon = np.array([21.3, 21.5, 20.9]), np.array([-157.9, -158.1, -156.5])
    x, y, crs = to_utm(lat, lon)
    assert crs.to_epsg() == 32604  # UTM zone 4N
    lat2, lon2 = from_utm(x, y, crs)
    np.testing.assert_allclose(lat2, lat, atol=1e-6)
    np.testing.assert_allclose(lon2, lon, atol=1e-6)


def _track():
    t = pd.to_datetime("2023-06-05 10:00:00") + pd.to_timedelta([0, 10, 25, 30, 30, 50], unit="s")
    return pd.DataFrame({
        "t": t, "lat": np.linspace(55, 55.01, 6), "lon": np.linspace(10, 10.01, 6),
        "sog": [10.0, 10, 10, np.nan, 10, 10], "cog": 90.0, "heading": [90.0, 90, 511, 90, 90, 90],
    })


def test_preprocess_resamples_to_one_second_and_converts_units():
    out = preprocess(_track())
    # rows with NaN sog and heading=511 dropped; regular 1 s grid, first sample removed
    assert (out.index.to_series().diff().iloc[1:] == pd.Timedelta("1s")).all()
    assert out.index[0] == pd.Timestamp("2023-06-05 10:00:01")
    assert out.index[-1] == pd.Timestamp("2023-06-05 10:00:50")
    np.testing.assert_allclose(out["sog"], 10 * KNOTS_TO_MS)
    np.testing.assert_allclose(out["heading"], np.pi / 2)


def test_preprocess_without_resampling_keeps_clean_rows():
    out = preprocess(_track(), freq=None)
    assert len(out) == 4  # 6 rows - NaN sog - heading 511 (duplicate timestamp row is the NaN one)


def test_sample_time_is_most_common_step():
    assert sample_time(pd.to_datetime(["2020-01-01 00:00", "2020-01-01 00:01", "2020-01-01 00:02", "2020-01-01 00:05"])) == 60
