'''
Created on 2013-11-23

@author: Walter
'''

from Swarm1d import *;
import sys;

if __name__ == '__main__':
    
    def func1(x):
        return x[0]**2;
    
    def func2(x):
        return (x[0]-4.)**2;
        
    swarm = Swarm1D(20, 1);
    swarm.setDisplayParam(600, 600, 20, 0.1)
    swarm.setParam(2.0, 2.0, 0.8, [func1, func2]);
    swarm.initParticles([5.0]);
    #swarm.initReferenceSet();
    swarm.initReferenceSet(True, 'nondomSet1D.data', 'domSet1D.data', 'paretoPeak1D.data');
    #swarm.dumpDominatedSet('domSet1D.data');
    #swarm.dumpNondominatedSet('nondomSet1D.data');
    #swarm.dumpParetoPeak('paretoPeak1D.data');
    
    figFolder = sys.path[0] + "\\img1";
    swarm.plot(0, figFolder);
    
    #swarm.showCentroidHist = True;
    #swarm.showAverageFitness = True;
    #swarm.showMaximinOfCentroid = True;
    #swarm.showGlobalBestPosition = True;
    #swarm.showPercentOfNondominance = True;
    swarm.showPosVariance = True;
    #swarm.showFitVariance = True;    
    swarm.showHausdorffDist = True;
    
    swarm.showSwarmCentroid = True;
    
    runPlan = [10, 20, 30, 40, 50, 60, 70, 80];
    count = 0;
    for r in runPlan:
        for t in range(r):
            swarm.update();
            count += 1;
        swarm.plot(count, figFolder);
        
    

    
    