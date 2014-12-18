'''
Created on Dec 18, 2014

@author: hcmi
'''

import numpy as np 

def sensorModel(x, noise=None):
    obs = np.zeros(2)
    if noise!=None:
        obs[0] = np.sqrt((-100-x[0])**2 + (100-x[1])**2) + noise[0]
        obs[1] = np.sqrt((150-x[0])**2 + (90-x[1])**2) + noise[1]
    else:
        obs[0] = np.sqrt((-100-x[0])**2 + (100-x[1])**2)
        obs[1] = np.sqrt((150-x[0])**2 + (90-x[1])**2)
    return obs

def sensorModel2(x, noise=None):
    obs = np.zeros(4)
    if noise!=None:
        obs[0] = np.sqrt((-100-x[0])**2 + (100-x[1])**2) + noise[0]
        obs[1] = np.sqrt((150-x[0])**2 + (90-x[1])**2) + noise[1]
        obs[2] = np.sqrt((-100-x[0])**2 + (100-x[1])**2) + noise[2]
        obs[3] = np.sqrt((150-x[0])**2 + (90-x[1])**2) + noise[3]
    else:
        obs[0] = np.sqrt((-100-x[0])**2 + (100-x[1])**2)
        obs[1] = np.sqrt((150-x[0])**2 + (90-x[1])**2)
        obs[2] = np.sqrt((-100-x[0])**2 + (100-x[1])**2)
        obs[3] = np.sqrt((150-x[0])**2 + (90-x[1])**2)
    return obs

def sensorModel4(x, noise=None):
    obs = np.zeros(8)
    if noise!=None:
        obs[0] = np.sqrt((-100-x[0])**2 + (100-x[1])**2) + noise[0]
        obs[1] = np.sqrt((150-x[0])**2 + (90-x[1])**2) + noise[1]
        obs[2] = np.sqrt((-100-x[0])**2 + (100-x[1])**2) + noise[2]
        obs[3] = np.sqrt((150-x[0])**2 + (90-x[1])**2) + noise[3]
        obs[4] = np.sqrt((-100-x[0])**2 + (100-x[1])**2) + noise[4]
        obs[5] = np.sqrt((150-x[0])**2 + (90-x[1])**2) + noise[5]
        obs[6] = np.sqrt((-100-x[0])**2 + (100-x[1])**2) + noise[6]
        obs[7] = np.sqrt((150-x[0])**2 + (90-x[1])**2) + noise[7]
    else:
        obs[0] = np.sqrt((-100-x[0])**2 + (100-x[1])**2)
        obs[1] = np.sqrt((150-x[0])**2 + (90-x[1])**2)
        obs[2] = np.sqrt((-100-x[0])**2 + (100-x[1])**2)
        obs[3] = np.sqrt((150-x[0])**2 + (90-x[1])**2)
        obs[4] = np.sqrt((-100-x[0])**2 + (100-x[1])**2) 
        obs[5] = np.sqrt((150-x[0])**2 + (90-x[1])**2)
        obs[6] = np.sqrt((-100-x[0])**2 + (100-x[1])**2)
        obs[7] = np.sqrt((150-x[0])**2 + (90-x[1])**2)
    return obs
def motionModel(x, u):
    nextX = x[0] + u[1] * np.cos(u[0])
    nextY = x[1] + u[1] * np.sin(u[0])
    return [nextX, nextY]