'''
Created on Dec 18, 2014

@author: hcmi
'''

from ParticleFilter import *
from motionModels import *
import matplotlib.pyplot as plt

if __name__ == '__main__':
    
    PARTICLE_NUM = 100
    
    pf = ParticleFilter(PARTICLE_NUM, 2, 2, motionModel, sensorModel)
    
    n_pos = np.loadtxt('n_pos.txt')
    pos = np.loadtxt('pos.txt')
    n_obs = np.loadtxt('n_obs.txt')
    obs = np.loadtxt('obs.txt')
    
    DATA_LEN = n_obs.shape[0]
    
    d = np.random.normal(loc=5, scale=1, size=DATA_LEN)
    theta = np.random.uniform(np.pi/5 - np.pi/36, np.pi/5 + np.pi/36, DATA_LEN)

    INPUT = np.vstack([d, theta]).T
    
    print DATA_LEN
    
    estX = []
    estY = []
    for i in range(DATA_LEN):
        
        print "OBS " + str(obs[i, 0]) + " " + str(obs[i, 1]) 
        pf.update(obs[i,:], INPUT[i,:])
        estPos, estVar = pf.estimate(pf.particles)
        estX.append(estPos[0])
        estY.append(estPos[1])
        print "EST " + str(estPos[0]) + " " + str(estPos[1])
    
    fig1 = plt.figure()
    ax1 = fig1.add_subplot(111)
    ax1.plot(pos[:,0], pos[:,1], '--', estX, estY, '-')
    ax1.legend(["TRUE", "ESTIMATED"])
    ax1.set_title('motion')
    
    plt.show()
        
        
        
        