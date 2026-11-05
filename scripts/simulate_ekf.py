"""Simulate a vessel and track it with an EKF (was simulate_ekf.m and osserver_model_.m).

Scenarios:
  maneuver : 200 steps of 60 s, sudden speed/heading change at step 60 (simulate_ekf.m)
  observer : 100 steps of 180 s starting from a UTM position (osserver_model_.m)
"""

import argparse
from functools import partial

import matplotlib.pyplot as plt
import numpy as np

from ais_anomaly import motion_models as mm
from ais_anomaly.filters import EKF

SCENARIOS = {
    "maneuver": dict(dt=60, steps=200, x0=[0, 0, 0.5, np.pi / 4, 0, 0],
                     Q=np.diag([1e-5, 1e-5, 1e-5, 1e-5, 1e-12, 1e-12]), R=np.zeros((4, 4)), P0=np.zeros((6, 6)),
                     maneuver=(60, 0.7, -np.pi / 4)),
    "observer": dict(dt=180, steps=101, x0=[2.5e5, 2.7e6, 17.9, 4.31, 0, 0],
                     Q=np.diag([5, 5, 0.5, 0.1, 1, 1]), R=np.diag([1e-4] * 4), P0=1e-4 * np.eye(6),
                     maneuver=None),
}


def run(dt, steps, x0, Q, R, P0, maneuver, seed=6):
    rng = np.random.default_rng(seed)
    f = partial(mm.ct_accel, dt=dt)
    ekf = EKF(f, mm.measure, x0, P0, Q, R, h_jacobian=lambda _: mm.measure_jacobian(6))
    x = np.array(x0, dtype=float)
    truth, est = [x.copy()], [ekf.x.copy()]
    for k in range(1, steps):
        if maneuver and k == maneuver[0]:
            x[2], x[3] = maneuver[1], maneuver[2]
        x = f(x) + rng.multivariate_normal(np.zeros(6), Q)
        z = mm.measure(x) + rng.multivariate_normal(np.zeros(4), R)
        ekf.predict()
        ekf.correct(z)
        truth.append(x.copy())
        est.append(ekf.x.copy())
    return np.array(truth).T, np.array(est).T


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--scenario", choices=SCENARIOS, default="maneuver")
    p.add_argument("--seed", type=int, default=6)
    args = p.parse_args()

    truth, est = run(**SCENARIOS[args.scenario], seed=args.seed)
    print(f"mean position error: {np.hypot(*(truth[:2] - est[:2])).mean():.3f} m")

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
    ax1.plot(truth[0], truth[1], "g-", lw=2, label="true")
    ax1.plot(est[0], est[1], "b--", lw=2, label="EKF")
    ax1.set(xlabel="x [m]", ylabel="y [m]", title="Trajectory", aspect="equal")
    ax1.legend()
    ax1.grid(True)
    ax2.plot(est[4], "r--", label="turn rate")
    ax2.plot(est[5], "b--", label="acceleration")
    ax2.set(xlabel="step", title="Estimated turn rate and acceleration")
    ax2.legend()
    ax2.grid(True)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
