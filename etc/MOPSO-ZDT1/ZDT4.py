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
        for i in range(2, 10):
            sum += x[i]**2 - 10 * np.cos(4 * np.pi * x[i]);
        g = 1 + 10 * 9 + sum;
        h = 1 - np.sqrt(x[0]/g);           
        return g * h;
    
    figFolder = sys.path[0];
    figFolder = sys.path[0] + "\\zdt4";

    trial_time = 30;
    
    paretoX = np.arange(0.0,1.0,0.005);
    paretoY = np.zeros(len(paretoX));
    localParetoY = np.zeros(len(paretoX));
    paretoPos = [];
    for i in range(len(paretoX)):
        paretoY[i] = 1 - np.sqrt(paretoX[i]);
        localParetoY[i] = 1 - np.sqrt(paretoX[i]/1.25);
        fitPos = np.matrix(np.zeros((1,2), np.float));
        fitPos[0,0] = paretoX[i];
        fitPos[0,1] = paretoY[i];
        paretoPos.append(fitPos);
    
    for tt in range(trial_time):

        swarm = SwarmND(100, 10);
        swarm.setDisplayParam(600, 600, 20, 0.1)
        swarm.setParam(2.0, 2.0, 0.8, [func1, func2]);
        ws = [];
        ws.append([0.0, 1.0]);
        for i in range(1,10):
            ws.append([-5.0, 5.0]) 
        swarm.initParticles(ws);
            
        swarm.paretoX = paretoX;
        swarm.paretoY = paretoY;
        swarm.localParetoX = paretoX;
        swarm.localParetoY = localParetoY;
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
            
        swarm.logger.dump("ZDT4-"+str(tt)+"-", figFolder)
