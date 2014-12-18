'''
Created on Dec 18, 2014

@author: hcmi
'''

from motionModels import *
from PathGenerator01 import *
import numpy as np
import matplotlib.pyplot as plt

if __name__ == '__main__':
    
    PATH_LEN = 100
    INIT_POS = [6.0, 3.0]
    
    d = np.random.normal(loc=5, scale=1, size=PATH_LEN)
    theta = np.random.uniform(np.pi/5 - np.pi/36, np.pi/5 + np.pi/36, PATH_LEN)

    INPUT = np.vstack([d, theta]).T
    
    path_gnr = PathGenerator(motionModel, sensorModel, 1.0, 1.0)
    pos, n_pos, obs, n_obs = path_gnr.generate(PATH_LEN, INIT_POS, INPUT)
    

    fig1 = plt.figure()
    ax1 = fig1.add_subplot(111)
    ax1.plot(pos[:,0], pos[:,1], '--', n_pos[:,0], n_pos[:,1], '-')
    ax1.set_title('motion')
    
    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111)
    ax2.plot(obs[:,0], obs[:,1], '--', n_obs[:,0], n_obs[:,1], '-')
    ax2.set_title('observation')
    plt.show()
    
    np.savetxt('n_pos.txt', n_pos)
    np.savetxt('pos.txt', pos)
    np.savetxt('n_obs.txt', n_obs)
    np.savetxt('obs.txt', obs)