'''
Created on Sep 30, 2014

@author: daqing_yi
'''

from GeneticAlgorithm import *
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

def calcFitness(x, weights):

    reference_fitness = [0.0, 0.0]
    f_val = zdt1(x)
    g_val = np.zeros(2, np.float)
    for d in range(2):
        g_val[d] = weights[d] * np.abs(f_val[d] - reference_fitness[d])
    return np.max(g_val)

if __name__ == '__main__':
    
    
    weights = []
    #weights.append([0.3, 0.4])
    #weights.append([0.1, 0.9])
    for w1 in np.arange(0.1, 1.0, 0.1):
        for w2 in np.arange(0.1, 1.0, 0.1):
            weights.append([w1, w2])
        
    chi = 0.4
    phi_p = 1.2
    phi_g = 1.2
    
    initRange = [0.0, 1.0]
        
    
    pareto_set = []        
    for weight in weights:
        print weight
        ga = GeneticAlgorithm(500, initRange, 30, calcFitness, weight, 0.01, 0.01) 
    
        for i in range(500):
            ga.next()
            
        pareto_set.append(ga.population[0].genes)
        
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