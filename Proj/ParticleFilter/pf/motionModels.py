'''
Created on Dec 18, 2014

@author: hcmi
'''

import numpy as np 

def sensorModel(x):
    obs = np.zeros(2)
    obs[0] = np.sqrt((-100-x[0])**2 + (100-x[1])**2)
    obs[1] = np.sqrt((150-x[0])**2 + (90-x[1])**2)
    return obs

def motionModel(x, u):
    nextX = x[0] + u[1] * np.cos(u[0])
    nextY = x[1] + u[1] * np.sin(u[0])
    return [nextX, nextY]