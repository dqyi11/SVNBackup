'''
Created on Jan 26, 2014

@author: daqing_yi
'''

from SwarmND import *;
import numpy as np;
import sys;

if __name__ == '__main__':

    def func1(x):
        return 1 - np.exp(-4 * x[0]) * (np.sin(6 * np.pi *x[0])**6);
    
    def func2(x):
        f = 1 - np.exp(-4 * x[0]) * (np.sin(6 * np.pi *x[0])**6);
        sum = 0.0;
        for i in range(1, 10):
            sum += x[i];
        g = 1 + 9 * (sum / 9.0)**(0.25);
        h = 1 - (f/g)**2;           
        return g * h;
    
    figFolder = sys.path[0];
    figFolder = sys.path[0] + "\\zdt6";

    
    baseX = np.arange(0.0,1.0,0.005);
    paretoX = np.zeros(len(baseX));
    paretoPos = [];
    for i in range(len(baseX)):
        paretoX[i] = 1 - np.exp(-4 * baseX[i]) * (np.sin(6 * np.pi *baseX[i])**6);
    paretoY = np.zeros(len(paretoX));
    for i in range(len(paretoX)):
        paretoY[i] = 1 - (paretoX[i])**2;
        fitPos = np.matrix(np.zeros((1,2), np.float));
        fitPos[0,0] = paretoX[i];
        fitPos[0,1] = paretoY[i];
        paretoPos.append(fitPos);
    

    swarm = SwarmND(100, 10);
    swarm.setSampleSize(5000);
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
            count += 1;
        swarm.plot(count, figFolder);
        
