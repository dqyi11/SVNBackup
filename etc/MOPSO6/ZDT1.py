'''
Created on Jan 24, 2014

@author: daqing_yi
'''

from SwarmND import *;
import sys;

if __name__ == '__main__':

    def func1(x):
        return x[0];
    
    def func2(x):
        sum = 0.0;
        for i in range(1,29):
            sum += x[i];
        g = 1 + 9 * sum / 29.0;
        h = 1 - np.sqrt(x[0]/g);           
        return g * h;
    
    figFolder = sys.path[0];
    figFolder = sys.path[0] + "\\zdt1";
    
    swarm = SwarmND(50, 30);
    swarm.setDisplayParam(600, 600, 20, 0.1)
    swarm.setParam(2.0, 2.0, 0.8, [func1, func2]);
    ws = [];
    for i in range(30):
        ws.append(5.0) 
    swarm.initParticles(ws);
    
    runPlan = [10, 20, 30, 40];
    count = 0;
    for r in runPlan:
        for t in range(r):
            swarm.update();
            count += 1;
        swarm.plot(count, figFolder);
            
    
    
    
    