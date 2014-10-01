'''
Created on Sep 30, 2014

@author: daqing_yi
'''

from MOPSOd import *
import numpy as np
import matplotlib.pyplot as plt

def zdt1(x):
    dim = len(x)
    #print dim
    vals = np.zeros(2, np.float)
    vals[0] = x[0]
    su = np.sum(x) - x[0]
    
    #print su
    
    g = 1.0 + 9.0 * su / (dim - 1)
    #print g 
    
    if g == 0:
        vals[1] = 0.0
    else:
        vals[1] = g * (1.0 - np.sqrt(vals[0] / g))
    return vals

def showFit(swm):  
          
    val_x = []
    val_y = []
    for p in swm.particles:
        val = zdt1(p.pos)
        val_x.append(val[0])
        val_y.append(val[1])
        
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.scatter(val_x, val_y)
    plt.show()

if __name__ == '__main__':
    
    chi = 0.4
    phi_p = 0.9
    phi_g = 0.9

    
    initRange = []
    for d in range(30):
        initRange.append([0.0, 1.0])
    initRange = np.array(initRange)
        
        
    swm = Swarm(500, 30, 2, zdt1, [0.0, 0.0], [0.1, 0.9])
    swm.initSwarm(initRange, chi, phi_p, phi_g)
    
    showFit(swm)
    
    for i in range(50):
        swm.update()
        print swm.gbFitness
        
    print zdt1(swm.gbPos)
        
    showFit(swm)

        

    
            