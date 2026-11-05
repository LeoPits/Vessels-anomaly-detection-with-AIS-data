import numpy as np
import pandas as pd
import pytest

from ais_anomaly.geo import course_to_math_angle, from_utm, haversine, to_utm, wrap_angle
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


def test_course_to_math_angle():
    # AIS: clockwise from north. Models: counter-clockwise from east (UTM x axis).
    north, east, south, west = np.deg2rad([0, 90, 180, 270])
    np.testing.assert_allclose(course_to_math_angle([north, east, south, west]), [np.pi / 2, 0, -np.pi / 2, -np.pi], atol=1e-12)


def test_course_east_moves_along_utm_x():
    lat, lon = np.array([21.0, 21.0]), np.array([-158.0, -157.99])  # second point is east of the first
    x, y, _ = to_utm(lat, lon)
    psi = course_to_math_angle(np.deg2rad(90))
    step = np.array([np.cos(psi), np.sin(psi)])
    utm_step = np.array([x[1] - x[0], y[1] - y[0]]) / np.hypot(x[1] - x[0], y[1] - y[0])
    # equal up to the UTM meridian convergence (~0.36 deg here, 1 deg off the central meridian)
    np.testing.assert_allclose(step, utm_step, atol=0.02)


def test_wrap_angle():
    np.testing.assert_allclose(wrap_angle([0, np.pi / 2, 3 * np.pi / 2, -3 * np.pi / 2, 2 * np.pi + 0.1]), [0, np.pi / 2, -np.pi / 2, np.pi / 2, 0.1])


def _track():
    t = pd.to_datetime("2023-06-05 10:00:00") + pd.to_timedelta([0, 10, 25, 30, 30, 50], unit="s")
    return pd.DataFrame({
        "t": t, "lat": np.linspace(55, 55.01, 6), "lon": np.linspace(10, 10.01, 6),
        "sog": [10.0, 10, 10, np.nan, 10, 10], "cog": 90.0, "heading": [90.0, 90, 511, 90, 90, 90],
    })


def test_preprocess_resamples_to_one_second_and_converts_units():
    out = preprocess(_track())
    # regular 1 s grid, first sample removed
    assert (out.index.to_series().diff().iloc[1:] == pd.Timedelta("1s")).all()
    assert out.index[0] == pd.Timestamp("2023-06-05 10:00:01")
    assert out.index[-1] == pd.Timestamp("2023-06-05 10:00:50")
    np.testing.assert_allclose(out["sog"], 10 * KNOTS_TO_MS)
    np.testing.assert_allclose(out["cog"], np.pi / 2)
    np.testing.assert_allclose(out["heading"], np.pi / 2)  # interpolated over the 511 gap


def test_preprocess_without_resampling_keeps_clean_rows():
    out = preprocess(_track(), freq=None)
    assert len(out) == 5  # only the NaN-sog row goes; heading 511 just becomes NaN
    assert out["heading"].isna().sum() == 1


def test_preprocess_drops_not_available_sog_and_cog():
    df = _track()
    df.loc[1, "sog"] = 102.3
    df.loc[2, "cog"] = 360.0
    assert len(preprocess(df, freq=None)) == 3


def test_preprocess_interpolates_angles_across_north():
    t = pd.to_datetime("2023-06-05 10:00:00") + pd.to_timedelta([0, 10, 20, 30], unit="s")
    df = pd.DataFrame({"t": t, "lat": 55.0, "lon": 10.0, "sog": 5.0, "cog": [340.0, 350, 10, 20]})
    out = preprocess(df)
    mid = out.loc["2023-06-05 10:00:15", "cog"]
    assert np.cos(mid) > 0.99  # ~0 deg; a plain spline would pass through ~180 deg
    assert ((out["cog"] >= 0) & (out["cog"] < 2 * np.pi)).all()


def test_sample_time_is_most_common_step():
    assert sample_time(pd.to_datetime(["2020-01-01 00:00", "2020-01-01 00:01", "2020-01-01 00:02", "2020-01-01 00:05"])) == 60
