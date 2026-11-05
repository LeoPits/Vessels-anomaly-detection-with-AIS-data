import numpy as np
import pytest

from ais_anomaly import motion_models as mm

X6 = np.array([10.0, -5.0, 3.0, 0.7, 0.02, 0.1])


def test_ct_one_step_by_hand():
    x = mm.ct(X6, dt=2.0)
    np.testing.assert_allclose(x, [10 + 6 * np.cos(0.7), -5 + 6 * np.sin(0.7), 3.2, 0.74, 0.02, 0.1])


def test_cv_equals_ct_without_turn_and_acceleration():
    x = X6.copy()
    x[4:] = 0
    np.testing.assert_allclose(mm.cv(x, 5.0), mm.ct(x, 5.0))


def test_ct_accel_adds_half_a_dt_squared():
    dt = 3.0
    diff = mm.ct_accel(X6, dt)[:2] - mm.ct(X6, dt)[:2]
    np.testing.assert_allclose(diff, 0.1 * dt**2 / 2 * np.array([np.cos(0.7), np.sin(0.7)]))


def test_ct_jacobian_matches_finite_differences():
    np.testing.assert_allclose(mm.ct_jacobian(X6, 1.5), mm.numerical_jacobian(lambda x: mm.ct(x, 1.5), X6), atol=1e-6)


def test_ctrv_ext_keeps_state_order():
    x = np.array([0, 0, 2.0, 0.0, 0.0, 0.5, 0.0])
    nxt = mm.ctrv_ext(x, 1.0)
    assert nxt[2] == pytest.approx(2.5)  # speed grows by ev*dt
    assert nxt[3] == pytest.approx(0.0)  # heading unchanged
    assert nxt[0] == pytest.approx(2.5)


def test_measure_and_jacobian():
    np.testing.assert_array_equal(mm.measure(X6), X6[:4])
    np.testing.assert_array_equal(mm.measure_jacobian(6) @ X6, X6[:4])


def test_ct_matrix_tends_to_cv_for_small_turn():
    np.testing.assert_allclose(mm.ct_matrix(1.0, 1e-9), mm.cv_matrix(1.0), atol=1e-8)
