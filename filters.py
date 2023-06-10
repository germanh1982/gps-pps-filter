import numpy as np
import scipy
from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import KalmanFilter

class Kalman:
    def __init__(self):
        self._kalman = KalmanFilter (dim_x=2, dim_z=1)
        self._kalman.x = np.array([1.65e-8, 1e-12])    # initial values: position, velocity
        self._kalman.F = np.array([[1.,1.], [0.,1.]]) # state transition matrix
        self._kalman.H = np.array([[1.,0.]]) # measurement function
        self._kalman.P = np.array([[1000., 0.], [0., 1000.]]) # covariance matrix
        self._kalman.R = np.array([[10000.]]) # measurement noise
        self._kalman.Q = Q_discrete_white_noise(dim=2, dt=0.1, var=0.013) # process noise

    def feed(self, value):
        self._kalman.predict()
        self._kalman.update(value)
        return self._kalman.x

class Filter:
    def __init__(self, order):
        self._zi = np.zeros(order, dtype=float)

    def feed(self, value):
        out, self._zi = scipy.signal.lfilter(*self._coeff, [value], zi=self._zi)
        print(self._zi)
        return out

class Butter(Filter):
    def __init__(self, order, cutoff):
        self._coeff = scipy.signal.butter(order, cutoff)
        super().__init__(order)

class Bessel(Filter):
    def __init__(self, order, cutoff):
        self._coeff = scipy.signal.bessel(order, cutoff)
        super().__init__(order)

 
