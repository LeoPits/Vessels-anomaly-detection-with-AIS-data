"""Moving horizon estimation of a simulated vessel (was Moving_horizon_estimation/MHE.m)."""

import argparse

import matplotlib.pyplot as plt
import numpy as np

from ais_anomaly import motion_models as mm
from ais_anomaly.filters.mhe import mhe
from ais_anomaly.io import save_figure


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--steps", type=int, default=50)
    p.add_argument("--horizon", type=int, default=10)
    p.add_argument("--seed", type=int, default=0)
    args = p.parse_args()

    rng = np.random.default_rng(args.seed)
    dt, Q, R = 0.1, 0.01 * np.eye(6), 0.1 * np.eye(2)
    x = np.array([0, 0, 1, np.pi / 4, 0.1, 0])
    truth, Y = np.zeros((6, args.steps)), np.zeros((2, args.steps))
    for t in range(args.steps):
        truth[:, t] = x
        Y[:, t] = x[:2] + rng.multivariate_normal(np.zeros(2), R)
        x = mm.ct(x, dt)

    X_hat = mhe(Y, dt, args.horizon, Q, R, x0=truth[:, 0])
    print(f"mean position error: {np.nanmean(np.hypot(*(truth[:2] - X_hat[:2]))):.3f} (measurement noise std {np.sqrt(R[0, 0]):.3f})")

    plt.plot(truth[0], truth[1], "b", lw=2, label="true")
    plt.plot(Y[0], Y[1], "k.", label="measurements")
    plt.plot(X_hat[0], X_hat[1], "r--", lw=2, label="MHE")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.title("Moving horizon estimation (CasADi)")
    plt.legend()
    plt.grid(True)
    print("saved", save_figure(plt.gcf(), f"mhe_h{args.horizon}"))
    plt.show()


if __name__ == "__main__":
    main()
