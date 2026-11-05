"""Moving horizon estimation with CasADi (port of Moving_horizon_estimation/MHE.m).

Fixes over the MATLAB draft: measurements are real NLP parameters (the original
passed 'p' to a solver that declared none), the cost weights are the *inverse*
covariances, and the estimate returned is the last state of each window.
"""

import casadi as ca
import numpy as np


def _ct_casadi(x, dt):
    return ca.vertcat(
        x[0] + x[2] * dt * ca.cos(x[3]),
        x[1] + x[2] * dt * ca.sin(x[3]),
        x[2] + x[5] * dt,
        x[3] + x[4] * dt,
        x[4],
        x[5],
    )


def mhe(Y, dt, horizon, Q, R, x0):
    """Estimate the 6-state CT trajectory from position measurements.

    Y : (2, T) measured positions. Returns (6, T); the first ``horizon - 1``
    columns are NaN because no full window is available yet.
    """
    Y = np.asarray(Y, dtype=float)
    T, n = Y.shape[1], 6
    X = ca.SX.sym("X", n, horizon)
    P = ca.SX.sym("P", 2, horizon)
    Qi, Ri = np.linalg.inv(Q), np.linalg.inv(R)

    J = 0
    for i in range(horizon):
        e = P[:, i] - X[0:2, i]
        J += e.T @ Ri @ e
        if i < horizon - 1:
            w = X[:, i + 1] - _ct_casadi(X[:, i], dt)
            J += w.T @ Qi @ w

    solver = ca.nlpsol(
        "mhe", "ipopt",
        {"x": ca.reshape(X, -1, 1), "f": J, "p": ca.reshape(P, -1, 1)},
        {"ipopt.print_level": 0, "print_time": 0, "ipopt.sb": "yes"},
    )

    X_hat = np.full((n, T), np.nan)
    guess = np.tile(np.asarray(x0, dtype=float)[:, None], (1, horizon))
    for k in range(T - horizon + 1):
        window = Y[:, k:k + horizon]
        sol = solver(x0=guess.reshape(-1, order="F"), p=window.reshape(-1, order="F"))
        est = np.asarray(sol["x"]).reshape(n, horizon, order="F")
        X_hat[:, k + horizon - 1] = est[:, -1]
        guess = np.column_stack([est[:, 1:], np.asarray(_ct_casadi(est[:, -1], dt)).ravel()])
    return X_hat
