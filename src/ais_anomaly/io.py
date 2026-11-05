"""Loading AIS data (NOAA MarineCadastre CSV/zip, AIA parquet) and caching results."""

import pickle
import shutil
import urllib.request
import zipfile
from pathlib import Path

import pandas as pd

NOAA_URL = "https://coast.noaa.gov/htdata/CMSP/AISDataHandler/{d:%Y}/AIS_{d:%Y_%m_%d}.zip"
"""Daily US-wide AIS files (one-minute reports). Verified available for 2015-2023."""

# shortcut: repo root from the source location, valid for the editable install (pip -e .);
# pass out_dir explicitly if the package is ever installed normally
OUTPUTS = Path(__file__).resolve().parents[2] / "outputs"


def save_figure(fig, name, out_dir=OUTPUTS):
    """Save a matplotlib figure (.png), folium map or holoviews/hvplot plot (.html).

    Returns the written path, e.g. ``outputs/02_waypoint_hulls.png``.
    """
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    if hasattr(fig, "savefig"):
        path = out_dir / f"{name}.png"
        fig.savefig(path, dpi=150, bbox_inches="tight")
    elif type(fig).__module__.startswith("folium"):
        path = out_dir / f"{name}.html"
        fig.save(path)
    else:
        import holoviews as hv  # only needed for interactive plots

        path = out_dir / f"{name}.html"
        hv.save(fig, path)
    return path


def filter_bbox(df, lat_min, lat_max, lon_min, lon_max):
    """Keep rows inside the bounding box (inclusive)."""
    return df[df["LAT"].between(lat_min, lat_max) & df["LON"].between(lon_min, lon_max)]


def read_zip(zip_path, lat_min, lat_max, lon_min, lon_max):
    """Read every CSV in a NOAA zip and keep the rows inside the bounding box.

    Was ``estrai_dataframe_da_zip`` in the notebook; reads the CSVs straight
    from the archive instead of extracting them to a temporary folder.
    """
    # read in chunks: a national daily CSV is ~7M rows
    with zipfile.ZipFile(zip_path) as zf:
        frames = [
            filter_bbox(chunk, lat_min, lat_max, lon_min, lon_max)
            for name in zf.namelist()
            if name.endswith(".csv")
            for chunk in pd.read_csv(zf.open(name), chunksize=1_000_000)
        ]
    return pd.concat(frames, ignore_index=True)


def read_zip_folder(folder, lat_min, lat_max, lon_min, lon_max):
    """``read_zip`` on every zip under ``folder`` (was ``estrai_dataframe_da_cartella``)."""
    frames = [read_zip(p, lat_min, lat_max, lon_min, lon_max) for p in sorted(Path(folder).rglob("*.zip"))]
    if not frames:
        raise FileNotFoundError(f"no .zip files under {folder}")
    return pd.concat(frames, ignore_index=True)


def read_noaa(path):
    """Read NOAA AIS data (.csv or .parquet), drop duplicate rows and parse BaseDateTime."""
    path = Path(path)
    df = pd.read_parquet(path) if path.suffix == ".parquet" else pd.read_csv(path)
    df = df.drop_duplicates(keep="first", ignore_index=True)
    df["BaseDateTime"] = pd.to_datetime(df["BaseDateTime"])
    return df


def download_noaa_day(day, bbox, out_dir, keep_zip=False):
    """Download one NOAA daily zip, keep the reports inside ``bbox`` and save them.

    ``bbox`` is ``(lat_min, lat_max, lon_min, lon_max)``. The filtered day is
    written to ``out_dir/AIS_YYYY_MM_DD.parquet``; if that file already exists
    nothing is downloaded, so an interrupted run can simply be restarted.
    Returns the parquet path.
    """
    day, out_dir = pd.Timestamp(day), Path(out_dir)
    out = out_dir / f"AIS_{day:%Y_%m_%d}.parquet"
    if out.exists():
        return out
    out_dir.mkdir(parents=True, exist_ok=True)
    zip_path = out_dir / f"AIS_{day:%Y_%m_%d}.zip"
    if not zip_path.exists():
        part = zip_path.with_suffix(".part")
        with urllib.request.urlopen(NOAA_URL.format(d=day), timeout=120) as r, part.open("wb") as f:
            shutil.copyfileobj(r, f, length=1 << 20)
        part.rename(zip_path)  # only complete downloads get the .zip name
    df = read_zip(zip_path, *bbox)
    df.to_parquet(out.with_suffix(".tmp"), index=False)
    out.with_suffix(".tmp").rename(out)
    if not keep_zip:
        zip_path.unlink()
    return out


def read_vessel_parquet(path, vid):
    """Rows of one vessel from an AIA parquet (MATLAB rowfilter on ``vid``)."""
    return pd.read_parquet(path, filters=[("vid", "==", vid)])


def cached(path, compute):
    """Load the pickle at ``path`` if present, else run ``compute()`` and save it."""
    path = Path(path)
    if path.exists():
        with path.open("rb") as f:
            return pickle.load(f)
    result = compute()
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("wb") as f:
        pickle.dump(result, f)
    return result
