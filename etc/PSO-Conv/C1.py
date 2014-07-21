'''
Created on 2014-3-19

@author: Walter
'''
from Swarm import *
import matplotlib.pyplot as plt

if __name__ == '__main__':
    
    def DeJongF4(X):
        val = 0.0
        for d in range(30):
            val += (d+1) * ( X[0,d]**4 )
        return val
    
    bounds = []
    bests = []
    pos = []
    swarm = Swarm(20, 30)
    swarm.setParam(20.0, 20.0, 0.8, DeJongF4)
    ws = []
    for i in range(30):
        ws.append([-20.0, 20.0]) 
    swarm.initParticles(ws)
    
    runPlan = [250]
    for r in runPlan:
        for t in range(r):
            swarm.update()
            
            bounds.append(swarm.bound)
            bests.append(swarm.globalbestFitness)
            pos.append(swarm.particles[0].pos[0,0]);
            
    dataLen = len(bounds)
    fig = plt.figure()
    ax1 = fig.add_subplot(111)
    ax1.plot(np.arange(dataLen), bounds,'r',label="bound")
    ax1.legend(loc=1)
    ax1.set_title("DeJongF4")
    ax1.set_ylabel('bound value')
    ax1.set_xlabel('runs')
    ax2 = ax1.twinx() # this is the important function
    ax2.plot(np.arange(dataLen), bests, 'g',label = "best")
    ax2.legend(loc=2)
    #ax2.set_xlim([0, np.e]);
    ax2.set_ylabel('best value')
    
    
    fig2 = plt.figure()
    ax3 = fig2.add_subplot(111)
    ax3.plot(np.arange(dataLen), pos)
    ax3.set_title("chi = 0.8, phi = 40")
    plt.show()
            
    
            
        
    