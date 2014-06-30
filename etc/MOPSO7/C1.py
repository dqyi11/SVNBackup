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
    
    
    trial_time = 30;
      
    for tt in range(trial_time):
            
        swarm = Swarm1D(50, 1);
        swarm.setDisplayParam(600, 600, 20, 0.1)
        swarm.setParam(2.0, 2.0, 0.8, [func1, func2]);
        swarm.initParticles([5.0]);
        #swarm.initReferenceSet();
        swarm.initReferenceSet(True, 'nondomSet1D.data', 'domSet1D.data');
        #swarm.dumpDominatedSet('domSet1D.data');
        #swarm.dumpNondominatedSet('nondomSet1D.data');
        
        figFolder = sys.path[0] + "\\img1";
        swarm.plot(0, figFolder);
        
        runPlan = [10, 40, 60, 100];
        count = 0;
        for r in runPlan:
            for t in range(r):
                swarm.update();
                count += 1;
            if tt == 0:
                swarm.plot(count, figFolder);
            
        swarm.logger.dump("C1-"+str(tt)+"-", figFolder)
    
    
    
    