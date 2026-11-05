"""EKF on a real AIS track to estimate speed, course, turn rate and acceleration.

Was Learning_motion_patterns.m (CT model), Copy_of_Learning_motion_patterns.m
(CTRV-ext model) and the "LEARNING MOTION PATTERNS ..." live scripts.

  --parquet PATH --vid ID    AIA parquet (Denmark), resampled at 1 s
  --noaa PATH --mmsi ID      NOAA data, .csv or .parquet (scripts/download_noaa.py)

The measured angle is COG (direction of motion) converted to the models'
convention (counter-clockwise from east); the MATLAB scripts used the raw AIS
heading, which is clockwise from north and often missing (511).
"""

import argparse
from functools import partial

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from ais_anomaly import motion_models as mm
from ais_anomaly.filters import EKF
from ais_anomaly.geo import course_to_math_angle, from_utm, to_utm, wrap_angle
from ais_anomaly.io import read_noaa, read_vessel_parquet, save_figure
from ais_anomaly.preprocessing import NOAA_COLUMNS, preprocess, sample_time

MODELS = {
    # (transition, analytic Jacobian or None, number of states, process noise diagonal)
    "ct": (mm.ct, mm.ct_jacobian, 6, [1e-5, 1e-5, 1e-5, 1e-5, 1e-6, 1e-6]),
    "ctrv": (mm.ctrv_ext, None, 7, [1e-5, 1e-5, 1e-5, 1e-5, 1e-8, 1e-8, 1e-8]),
}


def load_track(args):
    if args.parquet:
        return preprocess(read_vessel_parquet(args.parquet, args.vid).reset_index())
    df = read_noaa(args.noaa)
    # NOAA data is already 1-minute and has long gaps: no spline resampling (as in the .mlx)
    return preprocess(df[df["MMSI"] == args.mmsi].rename(columns=NOAA_COLUMNS), freq=None)


def run(track, model):
    f, jac, n, q = MODELS[model]
    dt = sample_time(track.index)
    x_utm, y_utm, crs = to_utm(track["lat"].to_numpy(), track["lon"].to_numpy())
    z = np.vstack([x_utm, y_utm, track["sog"].to_numpy(), course_to_math_angle(track["cog"].to_numpy())])
    x0 = np.r_[z[:, 0], np.zeros(n - 4)]
    ekf = EKF(partial(f, dt=dt), mm.measure, x0, 1e-5 * np.eye(n), np.diag(q), np.zeros((4, 4)),
              f_jacobian=partial(jac, dt=dt) if jac else None, h_jacobian=lambda _: mm.measure_jacobian(n), residual=mm.measure_residual)
    est = np.zeros((n, z.shape[1]))
    est[:, 0] = x0
    for k in range(1, z.shape[1]):
        # same order as the MATLAB scripts: correct with z_k, then predict k+1
        ekf.correct(z[:, k])
        est[:, k] = ekf.x
        ekf.predict()
    return z, est, crs


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument("--parquet")
    src.add_argument("--noaa")
    p.add_argument("--vid", type=int, default=244090800)
    p.add_argument("--mmsi", type=int, default=245206000)
    p.add_argument("--model", choices=MODELS, default="ct")
    args = p.parse_args()

    track = load_track(args)
    z, est, crs = run(track, args.model)
    hours = (track.index - track.index[0]) / pd.Timedelta("1h")
    lat_e, lon_e = from_utm(est[0], est[1], crs)
    print(f"{len(track)} samples, estimated track from ({lat_e[0]:.4f}, {lon_e[0]:.4f}) to ({lat_e[-1]:.4f}, {lon_e[-1]:.4f})")

    fig, ax = plt.subplots(2, 2, figsize=(12, 9))
    ax[0, 0].plot(z[0], z[1], "r", lw=3, label="measured")
    ax[0, 0].plot(est[0], est[1], "b--", lw=2, label="estimated")
    ax[0, 0].set(xlabel="x [m]", ylabel="y [m]", title="Trajectory (UTM)")
    ax[0, 1].plot(hours, z[2], "r", label="measured")
    ax[0, 1].plot(hours, est[2], "b--", label="estimated")
    ax[0, 1].set(xlabel="t [h]", ylabel="speed [m/s]", title="Speed")
    ax[1, 0].plot(hours, z[3], "r", label="measured")
    ax[1, 0].plot(hours, wrap_angle(est[3]), "b--", label="estimated")
    ax[1, 0].set(xlabel="t [h]", ylabel="course [rad, from east]", title="Course over ground")
    ax[1, 1].plot(hours, est[4], label="turn rate [rad/s]")
    ax[1, 1].plot(hours, est[5], label="acceleration [m/s²]" if args.model == "ct" else "speed drift")
    ax[1, 1].set(xlabel="t [h]", title="Estimated turn rate and acceleration")
    for a in ax.flat:
        a.legend()
        a.grid(True)
    plt.tight_layout()
    print("saved", save_figure(fig, f"motion_patterns_{args.model}_{args.vid if args.parquet else args.mmsi}"))
    plt.show()


if __name__ == "__main__":
    main()
