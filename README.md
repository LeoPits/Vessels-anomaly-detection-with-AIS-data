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
pytest                                   # 34 tests, synthetic data only, ~10 s

# download NOAA AIS data for Hawaii (see "Data" below), then open the notebooks
python scripts/download_noaa.py --start 2020-01-01 --end 2020-03-30 --region hawaii_2020Q1
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
| `notebooks/02_route_extraction.ipynb` | `data/hawaii_2020Q1.parquet` | `python scripts/download_noaa.py --start 2020-01-01 --end 2020-03-30 --region hawaii_2020Q1` (90 files, ~22 GB downloaded, ~100 MB kept) |
| `notebooks/01_extract_ais_data.ipynb` | zips in `data/raw/hawaii_2021_01/` | `python scripts/download_noaa.py --start 2021-01-01 --end 2021-01-31 --region hawaii_2021_01 --keep-zips` |
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
  routes.py          waypoints (DBSCAN + convex hull), route extraction, KDE
  motion_models.py   CT, CT+acceleration, CV, CTRV-ext, linear CV/CT for IMM, Jacobians
  filters/           EKF, IMM, MHE (CasADi)
scripts/             command-line entry points (see --help)
notebooks/           exploratory notebooks, outputs stripped
tests/               pytest, synthetic data
```

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
