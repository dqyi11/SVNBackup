'''
Created on Oct 18, 2014

@author: daqing_yi
'''

def zdt1_func(x):
    y = np.zeros(2, np.float)
    
    y[0] = x[0]
    
    sum = 0.0
    for i in range(1, 30):
        sum += x[i];
    g = 1 + 9 * (sum / 29.0);
    h = 1 - np.sqrt(x[0]/g);           
    y[1] = g * h;
    return y

from moead import *
from Visualization import *

if __name__ == '__main__':
    
    generation_number = 10
    population_size = 5
    neighbor_num = 30

    position_range = []
    for i in range(30):
        position_range.append([0.0, 1.0])
        
    moead = MOEAD(2, 30, zdt1_func)
    moead.initPopulation(population_size, neighbor_num, position_range)
    
    moead.run(generation_number)
    
    paretoX = np.arange(0.0,1.0,0.005);
    paretoY = np.zeros(len(paretoX));
    paretoPos = [];
    for i in range(len(paretoX)):
        paretoY[i] = 1 - np.sqrt(paretoX[i]);
    paretoFront = np.vstack((paretoX, paretoY))
    
    fitnessX = []
    fitnessY = []
    for p in moead.population:
        fitnessX.append(p.fitness[0])
        fitnessY.append(p.fitness[1])
    populationFitness = np.vstack((fitnessX, fitnessY))
    
    VisualizeParetoFront(populationFitness, paretoFront)