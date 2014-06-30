'''
Created on Nov 25, 2013

@author: daqing_yi
'''

from Swarm2d import *;
import sys;

if __name__ == '__main__':
    
    def func1(x):
        return x[0]**2 + x[1]**2;
    
    def func2(x):
        return 1 - (x[0]+np.pi)**2 / (4 * np.pi**2) + np.abs(x[1] - 5 * np.cos(x[0]))**(1.0/3.0);
        
    swarm = Swarm2D(100, 2);
    swarm.setDisplayParam(600, 600, 20, 0.1)
    swarm.setParam(1.0, 1.0, 0.6, [func1, func2]);
    swarm.initParticles([5.0, 5.0]);
    
    #swarm.initReferenceSet();
    swarm.initReferenceSet(True, 'nondomSet2D-2.data', 'domSet2D-2.data');
    #swarm.dumpDominatedSet('domSet2D-2.data');
    #swarm.dumpNondominatedSet('nondomSet2D-2.data');
    
    figFolder = sys.path[0] + "\\img3";
    swarm.plot(0, figFolder);
   
    
    swarm.showCentroidHist = True;
    swarm.showAverageFitness = True;
    swarm.showMaximinOfCentroid = True;
    swarm.showGlobalBestPosition = True;   
    swarm.showPercentOfNondominance = True;
    swarm.showPosVariance = True;
    swarm.showFitVariance = True;
    swarm.showHausdorffDist = True;
    
    
    runPlan = [10, 20, 30, 40];
    count = 0;
    for r in runPlan:
        for t in range(r):
            swarm.update();
            count += 1;
        swarm.plot(count, figFolder);
    