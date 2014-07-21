'''
Created on 2014-3-19

@author: Walter
'''

import numpy as np
import matplotlib.pyplot as plt

if __name__ == '__main__':
    
    gb_bound = 3.0
    pb_bound = 1.0
    
    runs = 500
    
    phi1 = 1
    phi2 = 1
    inertia = 0.1
    '''
    phi1 = 2.05    
    phi2 = 2.05
    inertia = 0.72984
    '''

    pos = np.random.random()  * 5
    vel = 0.0
    
    u_b = np.linalg.norm([pb_bound, gb_bound])
    x_0_b = np.linalg.norm([vel, pos])
    eig_A, eig_A_v = np.linalg.eig(np.array([[inertia, -inertia*(phi1+phi2)], [inertia, 1-inertia*(phi1+phi2)]]))
    eig_B, eig_B_v = np.linalg.eig([[inertia*phi1, inertia*phi2],[inertia*phi1, inertia*phi2]])
    
    A_b = np.max(np.abs(eig_A))
    B_b = np.max(np.abs(eig_B))
    
    print A_b
    print B_b
    
    pbHist = []
    gbHist = []
    posHist = []
    boundHist = []
    
    for i in range(runs):
        
        pb = pb_bound #* np.random.random() 
        gb = gb_bound #* np.random.random()
        
        pbHist.append(pb)
        gbHist.append(gb)
        posHist.append(pos)
        
        u1 = np.random.random()
        u2 = np.random.random()
        localForce = phi1 * u1 * (pb - pos)
        globalForce = phi2 * u2 * (gb - pos)
        
        vel = inertia * (vel + localForce + globalForce)
        pos = pos + vel
        
        boundHist.append( A_b**i + (1-A_b**(i+1))*B_b*u_b/(1-A_b) )
        
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(np.arange(runs), posHist)
    #ax.plot(np.arange(runs), pbHist, '.-', linewidth=5)
    #ax.plot(np.arange(runs), gbHist, '--', linewidth=5)
    ax.set_title(" $ \phi^{G} = $ " + str(phi1) + ", $ \phi^{P} = $ "+ str(phi2) + ", $ \chi = $ "+ str(inertia))
    #ax.legend(["position", "personal best", "global best"])
    ax.plot(np.arange(runs), boundHist)
    ax.legend(["position", "bounds"])
    
    plt.show()
    
    
        
    