"""Download NOAA MarineCadastre AIS data for a date range and a bounding box.

Each daily US-wide file (~250 MB zip) is downloaded, filtered to the bounding
box and saved as data/raw/<region>/AIS_YYYY_MM_DD.parquet; the zip is then
deleted. Days already done are skipped, so the command can be re-run after an
interruption. Finally all days are merged into data/<region>.parquet.

Examples:
  # January 2020, the data used by the notebooks
  python scripts/download_noaa.py --start 2020-01-01 --end 2020-01-31 --region hawaii_2020Q1 --out hawaii_2020_01
  # the whole first quarter (days already downloaded are skipped)
  python scripts/download_noaa.py --start 2020-01-01 --end 2020-03-30 --region hawaii_2020Q1
"""

import argparse
from pathlib import Path
from urllib.error import HTTPError

import pandas as pd

from ais_anomaly.io import download_noaa_day

HAWAII = (18.0, 22.5, -160.0, -150.0)  # lat_min, lat_max, lon_min, lon_max
DATA = Path(__file__).resolve().parents[1] / "data"


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--start", required=True, help="first day, YYYY-MM-DD")
    p.add_argument("--end", required=True, help="last day (inclusive), YYYY-MM-DD")
    p.add_argument("--region", default="hawaii", help="name of the output files")
    p.add_argument("--bbox", nargs=4, type=float, default=HAWAII, metavar=("LAT_MIN", "LAT_MAX", "LON_MIN", "LON_MAX"))
    p.add_argument("--keep-zips", action="store_true", help="keep the downloaded national zips")
    p.add_argument("--out", help="name of the merged file data/<out>.parquet (default: --region); "
                                 "e.g. merge only January from a Q1 region already downloaded")
    args = p.parse_args()

    days = pd.date_range(args.start, args.end, freq="D")
    raw = DATA / "raw" / args.region
    done = []
    for i, day in enumerate(days, 1):
        try:
            done.append(download_noaa_day(day, args.bbox, raw, keep_zip=args.keep_zips))
            print(f"[{i}/{len(days)}] {day:%Y-%m-%d} ok", flush=True)
        except HTTPError as e:
            print(f"[{i}/{len(days)}] {day:%Y-%m-%d} skipped: HTTP {e.code}", flush=True)

    if not done:
        raise SystemExit("nothing downloaded")
    out = DATA / f"{args.out or args.region}.parquet"
    pd.concat(map(pd.read_parquet, done), ignore_index=True).to_parquet(out, index=False)
    print(f"saved {out}")


if __name__ == "__main__":
    main()
