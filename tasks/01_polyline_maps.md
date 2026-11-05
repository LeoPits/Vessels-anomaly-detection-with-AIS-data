# Task 01: route maps with one line per trip

## Goal

`notebooks/02_route_extraction.ipynb` draws one `folium.RegularPolygonMarker` per AIS report. With a month of data a
route has tens of thousands of reports, so the maps become slow and the saved HTML huge. Draw **one
`folium.PolyLine` per trip** instead, through a small tested module.

## Setup (Windows, Git Bash or PowerShell)

- Work on a new branch from `python-work`: `git switch python-work && git switch -c cline/polyline-maps`
- Python environment: conda env `ais`. Run every Python command through conda, never with a bare `python`:
  `C:\dev\miniforge3\Scripts\conda.exe run -n ais python ...`
  (calling the env's `python.exe` directly crashes: native DLLs are not found).
- No data download is needed: tests use synthetic data.

## What to do

### 1. New module `src/ais_anomaly/maps.py`

Two functions, using only `folium` (already installed). No new dependencies.

```python
def waypoint_map(hulls, center=(21, -158), zoom=8):
    """Folium map with one GeoJson polygon per waypoint and a popup with its name.

    hulls: DataFrame with columns "name" (str) and "hull" (shapely geometry), as returned by
    ais_anomaly.routes.cluster_waypoints. Returns the folium.Map.
    """

def add_trips(m, df, trip_col="trip", color="blue", weight=2, popup_col=None):
    """Add one folium.PolyLine per trip to map m and return m.

    df has columns LAT, LON, BaseDateTime and trip_col. Points of each trip are sorted by
    BaseDateTime; trips with fewer than 2 points are skipped. Locations are [lat, lon].
    color: a color string, or a dict {trip_id: color}. popup_col: if given, the popup of each
    line shows the first value of that column in the trip (e.g. "VesselName").
    """
```

Keep the code short and readable; one-line docstrings are fine beyond the ones above.

### 2. Tests `tests/test_maps.py`

With small synthetic DataFrames (no files, no network):
- `add_trips` adds exactly one `folium.PolyLine` per trip with at least 2 points (count the children of the map
  whose type is `folium.PolyLine`); a trip with 1 point adds nothing.
- the PolyLine locations are `[lat, lon]` in time order even when the input rows are shuffled.
- `color` as a dict gives each trip its own color.
- `waypoint_map` returns a `folium.Map` with one `folium.GeoJson` per waypoint (use two small shapely Polygons).

### 3. Notebook `notebooks/02_route_extraction.ipynb`

**Edit the notebook only through a Python script that uses `nbformat`** (read, change `cell.source`, write).
Do not hand-edit the JSON. Do not execute the notebook and do not add outputs.

Change exactly these cells (find them by their content):

1. The cell with `Route_object = pd.concat([r for v in possible_vessel_route for r in routes.extract_routes(v)])`:
   give every trip an id:
   `Route_object = pd.concat([r.assign(trip=i) for i, r in enumerate(r for v in possible_vessel_route for r in routes.extract_routes(v))])`
2. The cell that defines `def waypoint_map(zoom):`: delete the local function and use
   `from ais_anomaly.maps import add_trips, waypoint_map` in the imports cell (the first code cell).
   Every call `waypoint_map(N)` becomes `waypoint_map(WP_dataframe_hull, zoom=N)`.
3. The cell with `Route_object["VesselName"] == "LADY BETTY"`: keep the vessel name and the color scale from blue
   (oldest) to red (newest), but color **per trip** (by each trip's first BaseDateTime) and draw it with
   `add_trips(m, subset_1, color={trip: color, ...})` instead of the marker loop. Keep `save_figure(m, "02_route_lady_betty")`.
4. The cell with `[("WP_3-WP_10", "green"), ("WP_10-WP_3", "red")]`: keep the two route names and colors,
   replace the marker loop with `add_trips(m, Route_object[Route_object["Route"] == route], color=color, popup_col="VesselName")`.
   Keep `save_figure(m, "02_routes_WP_3_WP_10")`.

Do not change anything else: route names, vessel names, parameters and the other cells stay as they are.

## Do not touch

- `src/ais_anomaly/routes.py`, `cleaning.py`, `preprocessing.py`, `geo.py`, `io.py`, `filters/`, `motion_models.py`
- notebooks `01_*` and `03_*`, `scripts/`, `README.md`, `environment.yml`, `data/`, `outputs/`

## Check before finishing

All three must succeed:

```
C:\dev\miniforge3\Scripts\conda.exe run -n ais python -m pytest -q
C:\dev\miniforge3\Scripts\conda.exe run -n ais python -c "import nbformat; nb = nbformat.read('notebooks/02_route_extraction.ipynb', as_version=4); [compile(c.source, str(i), 'exec') for i, c in enumerate(nb.cells) if c.cell_type == 'code' and not c.source.lstrip().startswith('%')]; assert not any(c.get('outputs') for c in nb.cells if c.cell_type == 'code'); print('notebook ok')"
git diff --stat python-work
```

The last command must list only: `src/ais_anomaly/maps.py`, `tests/test_maps.py`, `notebooks/02_route_extraction.ipynb`.

Then commit on `cline/polyline-maps` with a short message, and report: files changed, test output, anything you were
not sure about. Do not merge or push.
