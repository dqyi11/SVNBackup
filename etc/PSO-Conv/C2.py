'''
Created on 2014-3-19

@author: Walter
'''

from Swarm import *

if __name__ == '__main__':
    
    def Rosenbrock(X):
        val = 0.0
        for d in range(30-1):
            val += 100 * ( X[0,d+1] - X[0,d]**2 )**2 + (X[0,d]-1)**2
        return val
    
    bounds = []
    bests = []
    swarm = Swarm(20, 30)
    swarm.setParam(2.0, 2.0, 0.8, Rosenbrock)
    ws = []
    for i in range(30):
        ws.append([-100.0, 100.0]) 
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
    ax1.set_title("Rosenbrock")
    ax1.plot(np.arange(dataLen), bounds,'r',label="bound")
    ax1.legend(loc=1)
    ax1.set_ylabel('bound value')
    ax1.set_xlabel('runs')
    ax2 = ax1.twinx() # this is the important function
    ax2.plot(np.arange(dataLen), bests, 'g',label = "best")
    ax2.legend(loc=2)
    #ax2.set_xlim([0, np.e]);
    ax2.set_ylabel('best value')
    plt.show()