'''
Created on Dec 18, 2014

@author: hcmi
'''

from ParticleFilter import *
from motionModels import *
import matplotlib.pyplot as plt

if __name__ == '__main__':
    
    PARTICLE_NUM = 1000
    
    pf = ParticleFilter(PARTICLE_NUM, 2, 2, motionModel, sensorModel)
    
    pos = np.loadtxt('pos.txt')
    n_obs = np.loadtxt('n_obs1.txt')
    
    DATA_LEN = n_obs.shape[0]
    
    d = np.random.normal(loc=5, scale=1, size=DATA_LEN)
    #theta = np.random.uniform(np.pi/5 - np.pi/36, np.pi/5 + np.pi/36, DATA_LEN)
    theta = np.random.uniform(np.pi/5 - np.pi/10, np.pi/5 + np.pi/10, DATA_LEN)

    INPUT = np.zeros((DATA_LEN, 2), np.double)
    for i in range(DATA_LEN):
        INPUT[i,0] = theta[i]
        INPUT[i,1] = d[i]
        
    
    print DATA_LEN
    
    estX = []
    estY = []
    for i in range(DATA_LEN):
        print "OBS " + str(n_obs[i, 0]) + " " + str(n_obs[i, 1]) 
        pf.update(n_obs[i,:], INPUT[i,:])
        estPos, estVar = pf.estimate(pf.particles)
        estX.append(estPos[0])
        estY.append(estPos[1])
        print "EST " + str(estPos[0]) + " " + str(estPos[1])
        
        
    errX = []
    errY = []
    for i in range(DATA_LEN):
        errX.append(pos[i,0]-estX[i])
        errY.append(pos[i,1]-estY[i])
    
    fig1 = plt.figure()
    ax1 = fig1.add_subplot(111)
    ax1.plot(pos[:,0], pos[:,1], '--', estX, estY, '-')
    ax1.legend(["TRUE", "ESTIMATED"])
    ax1.set_title('motion')
    
    fig1 = plt.figure()
    ax1 = fig1.add_subplot(111)
    T = np.arange(DATA_LEN)
    ax1.plot(T, errX, 'g--', T, errY, 'b-')
    ax1.legend(["Error X", "Error Y"])
    ax1.set_title('Error')
    
    plt.show()
        
        
        
        