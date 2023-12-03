import numpy as np
from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import KalmanFilter

class Kalman:
    def __init__(self, initial_x, initial_v, meas_noise, proc_noise_var):
        self._kalman = KalmanFilter(dim_x=2, dim_z=1)
        self._kalman.x = np.array([initial_x, initial_v])    # initial values: position, velocity
        self._kalman.F = np.array([[1.,1.], [0.,1.]]) # state transition matrix
        self._kalman.H = np.array([[1.,0.]]) # measurement function
        self._kalman.P = np.array([[1000., 0.], [0., 1000.]]) # covariance matrix
        self._kalman.R = np.array([[meas_noise]]) # measurement noise
        self._kalman.Q = Q_discrete_white_noise(dim=2, dt=1, var=proc_noise_var) # process noise

    def feed(self, value):
        self._kalman.predict()
        self._kalman.update(value)
        return self._kalman.x

    @property
    def state(self):
        return self._kalman.x

