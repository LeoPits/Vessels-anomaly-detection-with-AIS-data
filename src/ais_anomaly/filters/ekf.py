"""Extended Kalman filter with additive noise.

Replaces MATLAB's ``extendedKalmanFilter`` as used in the original scripts:
``f(x)`` and ``h(x)`` are callables; Jacobians are optional and fall back to
finite differences, as MATLAB does.
"""

import numpy as np

from ..motion_models import numerical_jacobian


class EKF:
    def __init__(self, f, h, x0, P0, Q, R, f_jacobian=None, h_jacobian=None):
        self.f, self.h = f, h
        self.f_jacobian = f_jacobian or (lambda x: numerical_jacobian(f, x))
        self.h_jacobian = h_jacobian or (lambda x: numerical_jacobian(h, x))
        self.x = np.asarray(x0, dtype=float)
        self.P = np.asarray(P0, dtype=float)
        self.Q = np.asarray(Q, dtype=float)
        self.R = np.asarray(R, dtype=float)

    def predict(self):
        F = self.f_jacobian(self.x)
        self.x = np.asarray(self.f(self.x), dtype=float)
        self.P = F @ self.P @ F.T + self.Q
        return self.x

    def correct(self, z):
        H = self.h_jacobian(self.x)
        y = np.asarray(z) - self.h(self.x)
        S = H @ self.P @ H.T + self.R
        # pinv: the original scripts use zero R and P0, which can make S singular
        K = self.P @ H.T @ np.linalg.pinv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(self.x.size) - K @ H) @ self.P
        return self.x
