'''
Created on Dec 17, 2014

@author: daqing_yi
'''

import numpy

# Implements a linear Kalman filter.
class KalmanFilter:
    def __init__(self,_A, _B, _H, _x, _P, _Q, _R):
        self.A = _A                      # State transition matrix.
        self.B = _B                      # Control matrix.
        self.H = _H                      # Observation matrix.
        self.X_hat = _x # Initial state estimate.
        self.P_hat = _P  # Initial covariance estimate.
        self.Q = _Q                      # Estimated error in process.
        self.R = _R                      # Estimated error in measurements.
    def GetCurrentState(self):
        return self.X_hat
    def Step(self,U,measurement_vector):
        #---------------------------Prediction step-----------------------------
        predicted_state_estimate = self.A * self.X_hat + self.B * U
        predicted_prob_estimate = (self.A * self.P_hat) * numpy.transpose(self.A) + self.Q
        #--------------------------Observation step-----------------------------
        innovation = measurement_vector - self.H*predicted_state_estimate
        innovation_covariance = self.H*predicted_prob_estimate*numpy.transpose(self.H) + self.R
        #-----------------------------Update step-------------------------------
        kalman_gain = predicted_prob_estimate * numpy.transpose(self.H) * numpy.linalg.inv(innovation_covariance)
        self.X_hat = predicted_state_estimate + kalman_gain * innovation
        # We need the size of the matrix so we can make an identity matrix.
        size = self.P_hat.shape[0]
        # eye(n) = nxn identity matrix.
        self.P_hat = (numpy.eye(size)-kalman_gain*self.H)*predicted_prob_estimate