"""Vessel kinematic motion models (port of the MATLAB *stateModel* files).

Nonlinear models share the state ``[px, py, v, heading, omega, a]``
(position in metres, speed m/s, heading rad, turn rate rad/s, acceleration m/s^2).
``ctrv_ext`` uses 7 states: ``[px, py, v, heading, omega, ev, eomega]``.

Linear models for the IMM filter use ``[x, y, vx, vy]``.
"""

import numpy as np


# --- nonlinear models (EKF) -------------------------------------------------

def ct(x, dt):
    """Constant turn rate and acceleration, Euler step (stateModelCT.m)."""
    px, py, v, h, omega, a = x
    return np.array([
        px + v * dt * np.cos(h),
        py + v * dt * np.sin(h),
        v + a * dt,
        h + omega * dt,
        omega,
        a,
    ])


def ct_jacobian(x, dt):
    """Analytic Jacobian of ``ct`` (stateTransitionJacobianFcnCT.m)."""
    _, _, v, h, _, _ = x
    F = np.eye(6)
    F[0, 2] = dt * np.cos(h)
    F[0, 3] = -v * dt * np.sin(h)
    F[1, 2] = dt * np.sin(h)
    F[1, 3] = v * dt * np.cos(h)
    F[2, 5] = dt
    F[3, 4] = dt
    return F


def ct_accel(x, dt):
    """Like ``ct`` plus the a*dt^2/2 position term (root stateModel.m).

    The MATLAB original stored the state as [.., a, omega]; here it uses the
    common [.., omega, a] order.
    """
    px, py, v, h, omega, a = x
    return np.array([
        px + (v * dt + a * dt**2 / 2) * np.cos(h),
        py + (v * dt + a * dt**2 / 2) * np.sin(h),
        v + a * dt,
        h + omega * dt,
        omega,
        a,
    ])


def cv(x, dt):
    """Constant speed and heading (stateModelCV.m, which referenced an undefined vy_next)."""
    px, py, v, h, omega, a = x
    return np.array([px + v * dt * np.cos(h), py + v * dt * np.sin(h), v, h, omega, a])


def ctrv_ext(x, dt):
    """CTRV with speed/turn-rate drift terms (CTRV_ext_StateFcn.m).

    The MATLAB original read heading/speed as states 3/4 but wrote them back as
    4/3; this version keeps [px, py, v, heading, ...] on both sides.
    """
    px, py, v, h, omega, ev, eomega = x
    v_next = v + dt * ev
    omega_next = omega + dt * eomega
    h_next = h + dt * omega_next
    return np.array([
        px + dt * v_next * np.cos(h_next),
        py + dt * v_next * np.sin(h_next),
        v_next,
        h_next,
        omega_next,
        ev,
        eomega,
    ])


def measure(x):
    """AIS measures position, speed and heading: the first 4 states (measureModel.m)."""
    return np.asarray(x)[:4]


def measure_jacobian(n_states):
    """Jacobian of ``measure`` (measurementJacobianFcn.m)."""
    return np.eye(4, n_states)


def numerical_jacobian(f, x, eps=1e-6):
    """Central-difference Jacobian, used when no analytic one is given."""
    x = np.asarray(x, dtype=float)
    fx = np.asarray(f(x))
    J = np.empty((fx.size, x.size))
    for i in range(x.size):
        d = np.zeros_like(x)
        d[i] = eps
        J[:, i] = (np.asarray(f(x + d)) - np.asarray(f(x - d))) / (2 * eps)
    return J


# --- linear models (IMM), state [x, y, vx, vy] ------------------------------

def cv_matrix(dt):
    """Constant-velocity transition matrix."""
    return np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=float)


def ct_matrix(dt, omega):
    """Coordinated-turn transition matrix with known turn rate ``omega``."""
    s, c = np.sin(omega * dt), np.cos(omega * dt)
    return np.array([
        [1, 0, s / omega, -(1 - c) / omega],
        [0, 1, (1 - c) / omega, s / omega],
        [0, 0, c, -s],
        [0, 0, s, c],
    ])


def cv_noise(dt, sigma):
    """White-acceleration process noise for ``cv_matrix``."""
    return sigma**2 * np.array([
        [dt**4 / 4, 0, dt**3 / 2, 0],
        [0, dt**4 / 4, 0, dt**3 / 2],
        [dt**3 / 2, 0, dt**2, 0],
        [0, dt**3 / 2, 0, dt**2],
    ])


POSITION_H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=float)
"""IMM measurement matrix: only x, y are observed."""
