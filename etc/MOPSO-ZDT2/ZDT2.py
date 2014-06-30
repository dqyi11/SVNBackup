'''
Created on 2014-1-25

@author: Walter
'''

from SwarmND import *;
import numpy as np;
import sys;

if __name__ == '__main__':

    def func1(x):
        return x[0];
    
    def func2(x):
        sum = 0.0;
        for i in range(1, 30):
            sum += x[i];
        g = 1 + 9 * (sum / 29.0);
        h = 1 - (x[0]/g)**2;           
        return g * h;
    
    figFolder = sys.path[0];
    figFolder = sys.path[0] + "\\zdt2";
    
    trial_time = 30;
    
    paretoX = np.arange(0.0,1.0,0.005);
    paretoY = np.zeros(len(paretoX));
    paretoPos = [];
    for i in range(len(paretoX)):
        paretoY[i] = 1 - paretoX[i]**2;
        fitPos = np.matrix(np.zeros((1,2), np.float));
        fitPos[0,0] = paretoX[i];
        fitPos[0,1] = paretoY[i];
        paretoPos.append(fitPos);
    
    for tt in range(trial_time):
    
        swarm = SwarmND(50, 30);
        swarm.setDisplayParam(600, 600, 20, 0.1)
        swarm.setParam(2.0, 2.0, 0.8, [func1, func2]);
        ws = [];
        for i in range(30):
            ws.append([0.0, 1.0]) 
        swarm.initParticles(ws);

        swarm.paretoX = paretoX;
        swarm.paretoY = paretoY;
        swarm.paretoPos = paretoPos;
        
        runPlan = [30, 60, 80, 100];
        count = 0;
        for r in runPlan:
            for t in range(r):
                swarm.update();
                swarm.logger.log();
                count += 1;
            if tt == 0:
                swarm.plot(count, figFolder);
            
        swarm.logger.dump("ZDT2-"+str(tt)+"-", figFolder)
