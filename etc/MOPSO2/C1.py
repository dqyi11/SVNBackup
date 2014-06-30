'''
Created on 2013-11-23

@author: Walter
'''

from Swarm1d import *;

if __name__ == '__main__':
    
    def func1(x):
        return x[0]**2;
    
    def func2(x):
        return (x[0]-4.)**2;
        
    swarm = Swarm1D(100, 1);
    swarm.setDisplayParam(600, 600, 20, 0.1)
    swarm.setParam(2.0, 2.0, 0.8, [func1, func2]);
    swarm.initParticles([5.0]);
    swarm.initReferenceSet();
    
    swarm.plot(0);
    
    swarm.showCentroidHist = True;
    swarm.showAverageFitness = True;
    swarm.showMaximinOfCentroid = True;
    swarm.showGlobalBestPosition = True;
    
    swarm.maximinOnNondominatedOnly = True;
    
    runPlan = [10, 100, 200];
    count = 0;
    for r in runPlan:
        for t in range(r):
            swarm.update();
            count += 1;
        swarm.plot(count);

    
    