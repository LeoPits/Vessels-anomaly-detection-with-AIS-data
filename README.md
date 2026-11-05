# Vessels anomaly detection with AIS data

## Introduction
We take inspiration from TREAD* (Traffic Route Extraction and Anomaly Detection)[1], which
learns a statistical model for maritime traffic from AIS data in an unsupervised way.

> This is the Python-only branch (`python-work`). The original MATLAB code and notebooks are on `main`.

## Quick start

```bash
conda env create -f environment.yml      # creates the `ais` env and installs this package (pip -e .)
conda activate ais
nbstripout --install                     # strip notebook outputs on commit (filter is declared in .gitattributes)
pytest                                   # 49 tests, synthetic data only, ~10 s

# download NOAA AIS data for Hawaii (see "Data" below), then open the notebooks
python scripts/download_noaa.py --start 2020-01-01 --end 2020-01-31 --region hawaii_2020Q1 --out hawaii_2020_01
jupyter lab notebooks/
```

On Windows, run Python from an activated env (Miniforge Prompt, `conda activate ais`, or `conda run -n ais ...`): without activation the native libraries (PROJ, GDAL, CasADi) are not found.

## Data
National Oceanic and Atmospheric Administration: https://marinecadastre.gov/accessais

`scripts/download_noaa.py` downloads the daily US-wide files (`AIS_YYYY_MM_DD.zip`, ~250 MB each, one-minute reports),
keeps only the reports inside a bounding box (Hawaii by default, `--bbox LAT_MIN LAT_MAX LON_MIN LON_MAX` for another area)
and writes `data/<region>.parquet`. Each day is saved to `data/raw/<region>/` as soon as it is done, so an
interrupted download restarts where it stopped. `data/` is ignored by git.

| Notebook / script | Data it expects | Command |
|---|---|---|
| `notebooks/02_route_extraction.ipynb` | `data/hawaii_2020_01.parquet` (January 2020) | `python scripts/download_noaa.py --start 2020-01-01 --end 2020-01-31 --region hawaii_2020Q1 --out hawaii_2020_01` (31 files, ~7.5 GB downloaded, ~45 MB kept). For the whole quarter use `--end 2020-03-30` without `--out` and read `hawaii_2020Q1.parquet` |
| `notebooks/01_extract_ais_data.ipynb` | zips in `data/raw/hawaii_2021_01/` | `python scripts/download_noaa.py --start 2021-01-01 --end 2021-01-31 --region hawaii_2021_01 --keep-zips` |
| `notebooks/03_data_quality.ipynb` | `data/hawaii_2020_01.parquet` (or the days already in `data/raw/hawaii_2020Q1/`) | same download as notebook 02 |
| `scripts/learn_motion_patterns.py --noaa` | any NOAA `.parquet`/`.csv` | — |
| `scripts/track_imm.py --parquet`, `learn_motion_patterns.py --parquet` | AIA Denmark parquet (`denmark_2023-06-05_point-based.parquet`, not public) | — |

The daily files are verified available for 2015–2023; 2024 files are listed by NOAA but currently return 404.

The information contained in the AIS data (generally) are of dynamic and static type
![dataset](https://github.com/LeoPits/AIS_anomaly_detection/assets/19689590/61b3d2a2-d571-4d36-bea3-f4ad43aa7a58)

Area of interest: **Hawaii**

A bounding box is selected and it corresponds to the (chosen) specific area under surveillance
![image](https://github.com/LeoPits/AIS_anomaly_detection/assets/19689590/a52f0ff1-dbca-4253-9bdf-86fc5f8004c2)

## Repository layout

```
src/ais_anomaly/
  io.py              NOAA download, zip/CSV/parquet readers, pickle cache
  geo.py             haversine, lat/lon <-> UTM (pyproj)
  preprocessing.py   single-vessel cleaning and 1 s spline resampling
  cleaning.py        MMSI categories, impossible-jump filter
  routes.py          voyages, waypoints (DBSCAN + convex hull), route extraction, KDE
  maps.py            folium maps: waypoint polygons, one line per trip
  motion_models.py   CT, CT+acceleration, CV, CTRV-ext, linear CV/CT for IMM, Jacobians
  filters/           EKF, IMM, MHE (CasADi)
scripts/             command-line entry points (see --help)
notebooks/           exploratory notebooks, outputs stripped
tests/               pytest, synthetic data
data/                downloaded AIS data and cache (git-ignored)
outputs/             figures (PNG) and interactive maps (HTML) written by scripts and notebooks (git-ignored)
```

Every script and notebook saves its figures through `ais_anomaly.io.save_figure` into `outputs/`, e.g.
`02_waypoint_hulls.png`, `02_waypoints_map.html`, `track_imm_missing.png`. The DBSCAN result and the waypoint
assignment are cached in `data/cache/`: delete those files after changing the data or the parameters.

### Route pipeline (notebook 02)

Parameters chosen on January 2020 in `03_data_quality.ipynb` and set in the *Parameters* cell of `02_route_extraction.ipynb`:

1. **Cleaning**: ships only (`cleaning.ships_only`) and no impossible jumps (`cleaning.drop_jumps`, 50 kn).
2. **Voyages**: each vessel track is cut at silences longer than 60 min (`routes.split_by_vessel(max_gap="60min")`), voyages with >= 50 reports.
3. **Waypoints**: speed from positions, stationary reports (speed exactly 0) + first/last report of each voyage, DBSCAN (`eps=0.2` std, `min_samples=20`) and convex hulls `WP_n`.
4. **Routes**: runs of reports at sea between two waypoints `WP_a-WP_b`, loops `WP_a-WP_a` and open ends `SEA-WP_a` / `WP_a-SEA` (AIS coverage is lost offshore). On January 2020: 93 routes, 2,473 trips, 94.5 % of the traffic at sea.
5. **Anomaly scoring** (`routes.score_route`): KDE of each route with >= 10 trips (UTM metres); reports below the 5th percentile of density are anomalous; trips with >= 20 reports are ranked by their share of anomalous reports.

### Known limitations

- "Stationary" means a computed speed of exactly 0 m/s: a deliberate choice that keeps candidates and DBSCAN cost low. Moored vessels with GPS jitter are not candidates; above 0.25 kn waypoints appear in open sea (slow fishing), see `03_data_quality.ipynb`.
- A port visit is counted whenever a report falls inside a waypoint, even without stopping (1,166 pass-throughs vs 2,023 stops in January 2020).
- DBSCAN `eps` is in standard deviations, not metres: the south coast of Oahu (Honolulu, Pearl Harbor) is a single waypoint. Waypoint numbers `WP_n` change with the data and parameters, so the example routes in notebook 02 are named for January 2020.
- One KDE per route mixes vessel types: in January 2020 the most anomalous trips are cruise ships on routes travelled mostly by tugs and ferries. A KDE per route and vessel type is the natural next step.
- The KDE of a route includes the trip being scored, and is fitted on at most 5,000 sampled reports per route.
- `cleaning.drop_jumps` trusts the first report of each MMSI.
- The EKF scripts keep the MATLAB measurement noise `R = 0`; set a realistic `R` to estimate turn rate and acceleration on noisy data.

### Where the MATLAB code went

| MATLAB (on `main`) | Python |
|---|---|
| `stateModel*.m`, `CTRV_ext_StateFcn.m`, `measureModel.m`, `*Jacobian*.m` | `motion_models.py` |
| `extendedKalmanFilter` usage, `simulate_ekf.m`, `osserver_model_.m` | `filters/ekf.py`, `scripts/simulate_ekf.py` |
| `Tracking_IMM.m`, `IMM_REAL_DATA.m`, `normalize_utm.m` | `filters/imm.py`, `scripts/track_imm.py` |
| `Learning_motion_patterns.m`, `Copy_of_...m`, `LEARNING MOTION PATTERNS ....mlx` | `scripts/learn_motion_patterns.py` |
| `preprocess_data.m` | `preprocessing.py` |
| `MHE.m` | `filters/mhe.py`, `scripts/mhe_demo.py` |
| `deg2utm.m`, `utm2deg.m`, `ll2utm.m` | `geo.py` (pyproj) |

Bugs fixed in the port: `stateModelCV.m` used an undefined variable; `CTRV_ext_StateFcn.m` swapped speed and heading
between input and output; the IMM mixing ignored the mode probabilities; `MHE.m` passed measurements to a solver
without parameters and weighted the cost by covariances instead of their inverses; AIS heading 511 ("not available")
was treated as an angle; the AIS angle (clockwise from north) was fed to models that expect counter-clockwise from
east, so predictions moved the wrong way (now COG converted by `geo.course_to_math_angle`); the EKF heading innovation
and the spline resampling of angles did not handle the 359° → 1° wrap.

Waypoint clustering snaps candidates to a ~100 m grid (`decimals=3`) and weights each cell by its count before DBSCAN,
so three months of data fit in memory while density is counted as on the raw points.

Not ported: `gaussian_kernel.m` (unused; `scipy.stats.gaussian_kde` is used) and `untitled.mlx` (draft).
`Route_extraction.ipynb` and the Colab copy `Route extraction from ais data.ipynb` were older versions of
`02_route_extraction.ipynb`.

## Waypoints identification
Waypoints (WPs): identify either stationary points (ports, offshore platforms, etc.), entry points and exit points.

WPs identification is based on DBSCAN (Density-Based Spatial Clustering of Applications with Noise) method

Note: The hyperparameters of DBSCAN are tuned manually.

![image](https://github.com/LeoPits/AIS_anomaly_detection/assets/19689590/a9aaef02-f73f-4803-98a2-aba101fd8613)

## Route extraction
Once the waypoints are learned, a route can be built by clustering the extracted vessel flows, which connect:

* Two ports
* Entry point and port
* Port and Exit point
* Entry point and Exit point (i.e., transit routes)
![image](https://github.com/LeoPits/AIS_anomaly_detection/assets/19689590/4fb8a983-2791-4603-b7d3-30a3011d17f3)

## PoL learning (work in progress)
It is necessary to remove outliers from the routes to identify normal traffic.
In TREAD, it is used the stochastic method **Kernel Density Estimation (KDE)**.

![image](https://github.com/LeoPits/AIS_anomaly_detection/assets/19689590/c95ea577-764b-41f3-aef4-9402470dcaad)

## Motion models and filtering
Vessel kinematics are estimated from AIS tracks with an EKF on nonlinear motion models (CT, CTRV), an IMM filter
switching between constant-velocity and coordinated-turn models (with missing and irregular reports), and a moving
horizon estimator. Run `python scripts/<name>.py --help`.

## References
[1] *Pallotta, Giuliana, Vespe, Michele and Bryan, Karna. "Vessel pattern knowledge
discovery from AIS data: A framework for anomaly detection and route prediction."
Entropy 15.6 (2013): 2218-2245.
