"""IMM tracking with constant-velocity and coordinated-turn models.

Synthetic scenarios (was Tracking_IMM.m):
  basic     : regular 1 s sampling
  missing   : ~40% of the measurements dropped
  irregular : random AIS-like transmission intervals

Real data (was IMM_REAL_DATA.m):
  --parquet PATH --vid ID   one vessel from an AIA parquet (columns t, lat, lon, sog, cog, heading, vid)
"""

import argparse
from functools import partial

import matplotlib.pyplot as plt
import numpy as np

from ais_anomaly import motion_models as mm
from ais_anomaly.filters import imm_filter
from ais_anomaly.geo import to_utm
from ais_anomaly.io import read_vessel_parquet
from ais_anomaly.preprocessing import preprocess

SIGMA, OMEGA = 0.1, 0.1
TRANS = np.array([[0.9, 0.1], [0.1, 0.9]])


def models(sigma=SIGMA, omega=OMEGA):
    return [(mm.cv_matrix, partial(mm.cv_noise, sigma=sigma)),
            (partial(mm.ct_matrix, omega=omega), lambda dt: sigma**2 * np.eye(4))]


def synthetic(scenario, steps=100, seed=0):
    """True CT trajectory, measurements (NaN = missing) and time steps."""
    rng = np.random.default_rng(seed)
    dts = np.ones(steps)
    if scenario == "irregular":
        dts = 1 + 2 * rng.standard_normal(steps)
        dts[dts <= 0] = 1.0
    x = np.array([0, 0, 1.0, 1.0])
    truth, z = np.zeros((4, steps)), np.full((2, steps), np.nan)
    for k in range(steps):
        if k:
            x = mm.ct_matrix(dts[k], OMEGA) @ x + rng.multivariate_normal(np.zeros(4), SIGMA**2 * np.eye(4))
        truth[:, k] = x
        if scenario != "missing" or rng.random() > 0.4:
            z[:, k] = x[:2] + rng.normal(0, SIGMA, 2)
    return truth, z, dts


def real(parquet, vid):
    """Standardised UTM positions of one vessel, resampled at 1 s."""
    track = preprocess(read_vessel_parquet(parquet, vid).reset_index())
    x, y, _ = to_utm(track["lat"].to_numpy(), track["lon"].to_numpy())
    # standardised as in normalize_utm.m (MATLAB std uses N-1)
    x, y = (x - x.mean()) / x.std(ddof=1), (y - y.mean()) / y.std(ddof=1)
    v, h = track["sog"].to_numpy(), track["heading"].to_numpy()
    truth = np.vstack([x, y, v * np.cos(h), v * np.sin(h)])
    return truth, truth[:2].copy(), np.ones(truth.shape[1])


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--scenario", choices=["basic", "missing", "irregular"], default="basic")
    p.add_argument("--parquet", help="AIA parquet file (real-data mode)")
    p.add_argument("--vid", type=int, default=244090800, help="vessel id in the parquet")
    p.add_argument("--seed", type=int, default=0)
    args = p.parse_args()

    truth, z, dts = real(args.parquet, args.vid) if args.parquet else synthetic(args.scenario, seed=args.seed)
    est, mu = imm_filter(z, models(), TRANS, truth[:, 0], np.eye(4), [0.5, 0.5], mm.POSITION_H, SIGMA**2 * np.eye(2), dts)
    print(f"mean position error: {np.hypot(*(truth[:2] - est[:2])).mean():.3f}")

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 9))
    ax1.plot(truth[0], truth[1], "g-", lw=2, label="true")
    ax1.plot(z[0], z[1], "r.", label="measurements")
    ax1.plot(est[0], est[1], "b-", lw=2, label="IMM estimate")
    ax1.set(xlabel="x", ylabel="y", title="Position tracking with IMM")
    ax1.legend()
    ax2.plot(mu[0], "r-", label="P(CV)")
    ax2.plot(mu[1], "b-", label="P(CT)")
    ax2.set(xlabel="step", ylabel="probability", title="Mode probabilities")
    ax2.legend()
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
