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

from dmopso import *
from Visualization import *

if __name__ == '__main__':

    position_range = []
    for i in range(30):
        position_range.append([0.0, 1.0])
        
    chi = 0.8
    phi_p = 1.5
    phi_g = 1.5
    age_threshold = 3
    gamma = 0.5
    
    generation_number = 20
    population_size = 200
        
    dmopso = dMOPSO(2, 30, zdt1_func)
    dmopso.setParameters(chi, phi_p, phi_g, age_threshold, gamma)
    dmopso.initPopulation(population_size, position_range)

    
    dmopso.run(generation_number)
    
    paretoX = np.arange(0.0,1.0,0.005);
    paretoY = np.zeros(len(paretoX));
    paretoPos = [];
    for i in range(len(paretoX)):
        paretoY[i] = 1 - np.sqrt(paretoX[i]);
    paretoFront = np.vstack((paretoX, paretoY))
    
    fitnessX = []
    fitnessY = []
    for p in dmopso.population:
        fitnessX.append(p.fitness[0])
        fitnessY.append(p.fitness[1])
    populationFitness = np.vstack((fitnessX, fitnessY))
    
    VisualizeParetoFront(populationFitness, paretoFront)    
    