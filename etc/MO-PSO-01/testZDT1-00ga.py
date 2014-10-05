'''
Created on Oct 1, 2014

@author: daqing_yi
'''

from GeneticAlgorithm import *
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

def calcFitness(x, weights):

    reference_fitness = [0.0, 0.0]
    f_val = zdt1(x)
    g_val = np.zeros(2, np.float)
    for d in range(2):
        g_val[d] = weights[d] * np.abs(f_val[d] - reference_fitness[d])
    return np.max(g_val)

def showFit(ga):  
          
    val_x = []
    val_y = []
    for p in ga.population:
        #val = zdt1(p.genes)
        #val_x.append(val[0])
        #val_y.append(val[1])
        val_x.append(p.genes[0])
        val_y.append(p.genes[1])
        
        
    pf1 = np.arange(0.0, 1.0, 0.01)
    pf2 = 1.0 - np.sqrt(pf1)
        
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(pf1, pf2, 'r')
    ax.scatter(val_x, val_y)
    ax.plot(val_x[0], val_y[0], 'rs')
    plt.show()

if __name__ == '__main__':
    
    initRange = [0.0, 1.0]
    weight = [0.5, 0.5]
        
    ga = GeneticAlgorithm(500, initRange, 30, calcFitness, weight, 0.05, 0.05) 
    
    showFit(ga)
    
    for i in range(1000):
        ga.next()
        print ga.population[0].fitness
        
    print zdt1(ga.population[0].genes)
        
    showFit(ga)