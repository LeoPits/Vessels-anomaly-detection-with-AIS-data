import zipfile

import pandas as pd

from ais_anomaly import io

BBOX = (18, 22.5, -160, -150)


def _fake_noaa_zip(path):
    df = pd.DataFrame({
        "MMSI": [1, 1, 2, 3],
        "BaseDateTime": ["2021-01-01T00:00:00", "2021-01-01T00:00:00", "2021-01-01T00:01:00", "2021-01-01T00:02:00"],
        "LAT": [21.0, 21.0, 21.5, 40.0],  # vessel 3 is outside the box
        "LON": [-158.0, -158.0, -157.0, -70.0],
    })
    with zipfile.ZipFile(path, "w") as zf:
        zf.writestr("AIS_2021_01_01.csv", df.to_csv(index=False))


def test_download_day_filters_saves_and_resumes(tmp_path, monkeypatch):
    _fake_noaa_zip(tmp_path / "AIS_2021_01_01.zip")  # already downloaded: no network needed
    monkeypatch.setattr(io.urllib.request, "urlopen", lambda *a, **k: (_ for _ in ()).throw(AssertionError("network")))

    out = io.download_noaa_day("2021-01-01", BBOX, tmp_path)
    assert out.name == "AIS_2021_01_01.parquet"
    assert not (tmp_path / "AIS_2021_01_01.zip").exists()
    assert sorted(pd.read_parquet(out)["MMSI"]) == [1, 1, 2]

    assert io.download_noaa_day("2021-01-01", BBOX, tmp_path) == out  # second run: skipped


def test_read_noaa_parquet_drops_duplicates(tmp_path):
    _fake_noaa_zip(tmp_path / "AIS_2021_01_01.zip")
    out = io.download_noaa_day("2021-01-01", BBOX, tmp_path)
    df = io.read_noaa(out)
    assert len(df) == 2
    assert pd.api.types.is_datetime64_any_dtype(df["BaseDateTime"])


def test_noaa_url():
    assert io.NOAA_URL.format(d=pd.Timestamp("2021-01-05")).endswith("/2021/AIS_2021_01_05.zip")


def test_save_figure_png_and_folium(tmp_path):
    import folium
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, _ = plt.subplots()
    assert io.save_figure(fig, "plot", tmp_path).read_bytes()[:4] == b"\x89PNG"
    assert "leaflet" in io.save_figure(folium.Map(), "map", tmp_path).read_text(encoding="utf-8").lower()


def test_cached_computes_once(tmp_path):
    calls = []
    compute = lambda: calls.append(1) or {"a": 1}
    assert io.cached(tmp_path / "x.pkl", compute) == {"a": 1}
    assert io.cached(tmp_path / "x.pkl", compute) == {"a": 1}
    assert len(calls) == 1
