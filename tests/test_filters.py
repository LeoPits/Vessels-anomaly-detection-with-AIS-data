from functools import partial

import numpy as np

from ais_anomaly import motion_models as mm
from ais_anomaly.filters import EKF, imm_filter
from ais_anomaly.filters.mhe import mhe


def test_ekf_tracks_noisy_ct_trajectory():
    rng = np.random.default_rng(0)
    dt, N = 10.0, 150
    x = np.array([0, 0, 5.0, 0.3, 0.002, 0.0])
    R = np.diag([25.0, 25.0, 0.01, 1e-4])
    ekf = EKF(partial(mm.ct, dt=dt), mm.measure, x + [30, -30, 0, 0, 0, 0], np.eye(6), np.diag([1, 1, 1e-3, 1e-5, 1e-8, 1e-6]), R,
              f_jacobian=partial(mm.ct_jacobian, dt=dt), h_jacobian=lambda _: mm.measure_jacobian(6))
    errors = []
    for _ in range(N):
        x = mm.ct(x, dt)
        z = mm.measure(x) + rng.multivariate_normal(np.zeros(4), R)
        ekf.predict()
        ekf.correct(z)
        errors.append(np.hypot(*(ekf.x[:2] - x[:2])))
    assert np.mean(errors[-50:]) < 5.0  # below the 5 m measurement noise


def _imm_scenario(missing=False, seed=1):
    rng = np.random.default_rng(seed)
    sigma, omega, N = 0.1, 0.1, 100
    x = np.array([0, 0, 1.0, 1.0])
    truth, z = [], np.full((2, N), np.nan)
    for k in range(N):
        if k:
            x = mm.ct_matrix(1.0, omega) @ x + rng.normal(0, sigma, 4)
        truth.append(x)
        if not missing or rng.random() > 0.4:
            z[:, k] = x[:2] + rng.normal(0, sigma, 2)
    models = [(mm.cv_matrix, partial(mm.cv_noise, sigma=sigma)),
              (partial(mm.ct_matrix, omega=omega), lambda dt: sigma**2 * np.eye(4))]
    est, mu = imm_filter(z, models, [[0.9, 0.1], [0.1, 0.9]], truth[0], np.eye(4), [0.5, 0.5], mm.POSITION_H, sigma**2 * np.eye(2))
    return np.array(truth).T, est, mu


def test_imm_probabilities_and_tracking():
    truth, est, mu = _imm_scenario()
    np.testing.assert_allclose(mu.sum(axis=0), 1.0)
    assert mu[1, -20:].mean() > 0.5  # the truth is a coordinated turn
    assert np.hypot(*(est[:2] - truth[:2]))[10:].mean() < 0.5


def test_imm_handles_missing_measurements():
    truth, est, mu = _imm_scenario(missing=True)
    assert np.isfinite(est).all()
    assert np.hypot(*(est[:2] - truth[:2]))[10:].mean() < 1.0


def test_mhe_recovers_straight_line():
    rng = np.random.default_rng(2)
    dt, T = 1.0, 30
    x = np.array([0, 0, 2.0, np.pi / 4, 0, 0])
    Y = np.zeros((2, T))
    for t in range(T):
        Y[:, t] = x[:2] + rng.normal(0, 0.1, 2)
        x = mm.ct(x, dt)
    X_hat = mhe(Y, dt, horizon=10, Q=0.01 * np.eye(6), R=0.01 * np.eye(2), x0=[0, 0, 1, 0.5, 0, 0])
    assert np.isnan(X_hat[:, :9]).all()
    assert abs(np.nanmean(X_hat[2, 9:]) - 2.0) < 0.2
