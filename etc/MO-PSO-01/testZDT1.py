'''
Created on Sep 30, 2014

@author: daqing_yi
'''

from MOPSOd import *
import numpy as np
import matplotlib.pyplot as plt

def zdt1(x):
    dim = len(x)
    vals = np.zeros(2, np.float)
    vals[0] = x[0]
    su = np.sum(x) - x[0]
    g = 1 + 9 * su / (dim - 1)
    if g == 0:
        vals[1] = 0
    else:
        vals[1] = g * (1 - np.sqrt(vals[0] / g))
    return vals


if __name__ == '__main__':
    
    
    weights = []
    weights.append([0.3, 0.4])
    weights.append([0.1, 0.9])
    '''
    for w1 in np.arange(0.0, 1.0, 0.1):
        for w2 in np.arange(0.0, 1.0, 0.1):
            weights.append([w1, w2])
    '''
    
    
    chi = 0.4
    phi_p = 1.2
    phi_g = 1.2
    
    initRange = []
    for d in range(30):
        initRange.append([0.0, 1.0])
    initRange = np.array(initRange)
        
    
    pareto_set = []        
    for weight in weights:
        
        swm = Swarm(500, 30, 2, zdt1, [0.0, 0.0], weight)
        swm.initSwarm(initRange, chi, phi_p, phi_g)
        
        for i in range(50):
            swm.update()
            
        pareto_set.append(swm.gbPos)
        
    x_p = []
    y_p = []
    for ps in pareto_set:
        val = zdt1(ps)
        x_p.append(val[0])
        y_p.append(val[1])
        
    pf1 = np.arange(0.0, 1.0, 0.01)
    pf2 = 1.0 - np.sqrt(pf1)
    
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(pf1, pf2)
    ax.scatter(x_p, y_p)
    
    plt.show()