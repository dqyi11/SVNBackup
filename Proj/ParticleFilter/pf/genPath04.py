'''
Created on Dec 18, 2014

@author: hcmi
'''

from motionModels import *
from PathGenerator04 import *
import numpy as np
import matplotlib.pyplot as plt

if __name__ == '__main__':
    
    PATH_LEN = 100
    INIT_POS = [6.0, 3.0]
    
    d = np.random.normal(loc=5, scale=1, size=PATH_LEN)
    #theta = np.random.uniform(np.pi/5 - np.pi/36, np.pi/5 + np.pi/36, PATH_LEN)
    theta = np.random.uniform(np.pi/5 - np.pi/10, np.pi/5 + np.pi/10, PATH_LEN)

    INPUT = np.zeros((PATH_LEN, 2), np.double)
    for i in range(PATH_LEN):
        INPUT[i,0] = theta[i]
        INPUT[i,1] = d[i]
    
    path_gnr = PathGenerator(motionModel, sensorModel, 4.0)
    pos, n_obs1, n_obs2, n_obs3, n_obs4 = path_gnr.generate(PATH_LEN, INIT_POS, INPUT)
    
    
    fig1 = plt.figure()
    ax1 = fig1.add_subplot(111)
    ax1.plot(pos[:,0], pos[:,1], '--')
    ax1.set_title('motion')
    
    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111)
    ax2.plot(n_obs1[:,0], n_obs1[:,1], '-', n_obs2[:,0], n_obs2[:,1], '--')
    ax2.set_title('observation')
    plt.show()
    
    np.savetxt('pos.txt', pos)
    np.savetxt('n_obs1.txt', n_obs1)
    np.savetxt('n_obs2.txt', n_obs2)
    np.savetxt('n_obs3.txt', n_obs3)
    np.savetxt('n_obs4.txt', n_obs4)