'''
Created on 2014-3-19

@author: Walter
'''

import numpy as np
import matplotlib.pyplot as plt

if __name__ == '__main__':
    
    gb_bound = 3.0
    pb_bound = 3.0
    
    runs = 1000
    
    #phi1 = 2
    #phi2 = 2
    #inertia = 0.8
    phi1 = 0.5
    inertia = 0.5
    phi2 = 0.5
    
    pos = (np.random.random() - 0.5) * 20
    vel = 0.0
    
    pbHist = []
    gbHist = []
    posHist = []
    
    for i in range(runs):
        
        pb = pb_bound * np.random.random() 
        gb = gb_bound * np.random.random()
        
        pbHist.append(pb)
        gbHist.append(gb)
        posHist.append(pos)
        
        u1 = np.random.random()
        u2 = np.random.random()
        localForce = phi1 * u1 * (pb - pos)
        globalForce = phi2 * u2 * (gb - pos)
        
        vel = inertia * (vel + localForce + globalForce)
        pos = pos + vel
        
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(np.arange(runs), posHist, np.arange(runs), pbHist, np.arange(runs), gbHist)
    ax.legend(["position", "personal best", "global best"])
    
    plt.show()
    
    
        
    