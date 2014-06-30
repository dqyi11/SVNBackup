'''
Created on 2013-11-23

@author: Walter
'''

from Swarm2d import *;

if __name__ == '__main__':
    
    def func1(x):
        return x[0]**2 + x[1]**2;
    
    def func2(x):
        return (x[0]-4.)**2 + (x[1]+3.)**2;
        
    swarm = Swarm2D(100, 2);
    swarm.setDisplayParam(600, 600, 20, 0.1)
    swarm.setParam(1.0, 1.0, 0.6, [func1, func2]);
    swarm.initParticles([5.0, 5.0]);
    swarm.initReferenceSet();
    
    swarm.plot(0);
   
    swarm.showCentroidHist = True;
    swarm.showAverageFitness = True;
    swarm.showMaximinOfCentroid = True;
    swarm.showGlobalBestPosition = True;    
    
    runPlan = [10, 100, 200];
    count = 0;
    for r in runPlan:
        for t in range(r):
            swarm.update();
            count += 1;
        swarm.plot(count);