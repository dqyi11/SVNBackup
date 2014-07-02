'''
Created on 2014-3-19

@author: Walter
'''

from Swarm import *

if __name__ == '__main__':
    
    def Griewank(X):
        val = 0.0
        for d in range(30):
            val += X[0,d]**2 
        val /= 4000.0
        val2 = 0.0
        for d in range(30):
            val2 *= np.cos(X[0,d]/np.sqrt(float(d))) + 1
        return val - val2
    
    bounds = []
    bests = []
    swarm = Swarm(20, 30)
    swarm.setParam(2.0, 2.0, 0.8, Griewank)
    ws = []
    for i in range(30):
        ws.append([-600.0, 600.0]) 
    swarm.initParticles(ws)
    
    runPlan = [250]
    for r in runPlan:
        for t in range(r):
            swarm.update()
            #print swarm.globalbestFitness
            
            bounds.append(swarm.bound)
            bests.append(swarm.globalbestFitness)
            
    dataLen = len(bounds)
    fig = plt.figure()
    ax1 = fig.add_subplot(111)
    ax1.plot(np.arange(dataLen), bounds,'r',label="bound")
    ax1.legend(loc=1)
    ax1.set_title("Griewank")
    ax1.set_ylabel('bound value')
    ax1.set_xlabel('runs')
    ax2 = ax1.twinx() # this is the important function
    ax2.plot(np.arange(dataLen), bests, 'g',label = "best")
    ax2.legend(loc=2)
    #ax2.set_xlim([0, np.e]);
    ax2.set_ylabel('best value')
    plt.show()