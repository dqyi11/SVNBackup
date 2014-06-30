'''
Created on 2013-11-23

@author: Walter
'''

from Swarm2d import *;
import sys;

if __name__ == '__main__':
    
    def func1(x):
        return x[0]**2 + x[1]**2;
    
    def func2(x):
        return (x[0]-4.)**2 + (x[1]+3.)**2;
    
    
    trial_time = 30;
      
    for tt in range(trial_time):
        
        swarm = Swarm2D(50, 2);
        swarm.setDisplayParam(600, 600, 20, 0.1)
        swarm.setParam(1.0, 1.0, 0.6, [func1, func2]);
        swarm.initParticles([5.0, 5.0]);
        #swarm.initReferenceSet();
        swarm.initReferenceSet(True, 'nondomSet2D.data', 'domSet2D.data');
        #swarm.dumpDominatedSet('domSet2D.data');
        #swarm.dumpNondominatedSet('nondomSet2D.data');
        
        figFolder = sys.path[0] + "\\img2";
        swarm.plot(0, figFolder);
        
        runPlan = [10, 40, 60, 100];
        count = 0;
        for r in runPlan:
            for t in range(r):
                swarm.update();
                count += 1;
            if tt == 0:
                swarm.plot(count, figFolder);
            
        swarm.logger.dump("C2-"+str(tt)+"-", figFolder)
        
