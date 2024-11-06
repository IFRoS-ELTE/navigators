from typing import List

import numpy as np
from scipy.linalg import block_diag
from utils.common import COMPASS_UNCERTAINTY, GPS_UNCERTAINTY


class EKFMeasurement:
    def __init__(self, value):
        self.z = value
        self.used = False

        self.H = None
        self.V = None
        self.R = None

    def h(self, state: np.ndarray):
        return self.H @ np.array(state).reshape((-1, 1))


class CompassMeasurement(EKFMeasurement):
    def __init__(self, value):
        assert value.shape == (1, 1), "CompassMeasurement should have shape (1, 1)"
        super().__init__(value)
        self.H = np.array([0, 0, 1]).reshape((1, 3))
        self.V = np.array([1]).reshape((1, 1))
        self.R = np.array([COMPASS_UNCERTAINTY]).reshape((1, 1))


class LocationMeasurement(EKFMeasurement):
    def __init__(self, value):
        assert value.shape == (2, 1), "LocationMeasurement should have shape (2, 1)"

        super().__init__(value)
        self.H = np.array([[1, 0, 0], [0, 1, 0]]).reshape((2, 3))
        self.V = np.eye(2)
        self.R = np.diag(np.array(GPS_UNCERTAINTY))


def combine_measurements(ms: List[EKFMeasurement], state):
    """Combine a list of measurements, to be applied in a single EKF update step."""
    z = None
    R = None
    H = None
    V = None
    h = None

    for m in ms:
        z = m.z if z is None else np.concatenate((z, m.z))
        R = m.R if R is None else block_diag(R, m.R)
        H = m.H if H is None else np.concatenate((H, m.H))
        V = m.V if V is None else block_diag(V, m.V)
        h = m.h(state) if h is None else np.concatenate((h, m.h(state)))

    return z, R, H, V, h


if __name__ == "__main__":
    m1 = CompassMeasurement(np.array([0.1]).reshape((1, 1)))
    m2 = LocationMeasurement(np.array([1, 2]).reshape((2, 1)))

    print(combine_measurements((m1, m2)))
