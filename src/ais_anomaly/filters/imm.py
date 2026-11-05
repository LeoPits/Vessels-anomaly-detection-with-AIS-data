"""Interacting Multiple Model filter (port of Tracking_IMM.m / IMM_REAL_DATA.m).

The MATLAB loop mixed the model estimates with the raw transition matrix,
ignoring the mode probabilities; this is the standard IMM (Bar-Shalom).
"""

import numpy as np


def imm_filter(z, models, trans, x0, P0, mu0, H, R, dts=1.0):
    """Run an IMM filter over measurements ``z``.

    z      : (m, N) measurements; columns containing NaN are treated as missing.
    models : list of ``(F(dt), Q(dt))`` callables, one per mode.
    trans  : (M, M) Markov matrix, ``trans[i, j]`` = P(mode j at k | mode i at k-1).
    dts    : scalar or (N,) time steps, for irregular AIS sampling.

    Returns ``(x_est (n, N), mu_hist (M, N))``.
    """
    z = np.asarray(z, dtype=float)
    N, M, n = z.shape[1], len(models), len(x0)
    dts = np.broadcast_to(np.asarray(dts, dtype=float), (N,))
    trans = np.asarray(trans, dtype=float)

    xs = np.tile(np.asarray(x0, dtype=float), (M, 1))
    Ps = np.tile(np.asarray(P0, dtype=float), (M, 1, 1))
    mu = np.asarray(mu0, dtype=float)
    x_est, mu_hist = np.zeros((n, N)), np.zeros((M, N))

    for k in range(N):
        # mixing
        c_bar = trans.T @ mu
        w = trans * mu[:, None] / c_bar[None, :]  # w[i, j] = P(mode i | mode j next)
        x_mix = w.T @ xs
        P_mix = np.zeros_like(Ps)
        for j in range(M):
            for i in range(M):
                d = xs[i] - x_mix[j]
                P_mix[j] += w[i, j] * (Ps[i] + np.outer(d, d))

        # mode-matched Kalman filters
        L = np.ones(M)
        for j, (F_fn, Q_fn) in enumerate(models):
            F, Q = F_fn(dts[k]), Q_fn(dts[k])
            x = F @ x_mix[j]
            P = F @ P_mix[j] @ F.T + Q
            if not np.isnan(z[:, k]).any():
                y = z[:, k] - H @ x
                S = H @ P @ H.T + R
                K = P @ H.T @ np.linalg.inv(S)
                x = x + K @ y
                P = (np.eye(n) - K @ H) @ P
                L[j] = np.exp(-0.5 * y @ np.linalg.solve(S, y)) / np.sqrt(np.linalg.det(2 * np.pi * S))
            xs[j], Ps[j] = x, P

        # mode probabilities and combination
        mu = L * c_bar
        mu = mu / mu.sum() if mu.sum() > 0 else c_bar  # all likelihoods underflowed
        x_est[:, k] = mu @ xs
        mu_hist[:, k] = mu

    return x_est, mu_hist
