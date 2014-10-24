'''
Created on Oct 18, 2014

@author: daqing_yi
'''

from zdt import *
from moead import *
from Visualization import *

if __name__ == '__main__':
    
    generation_number = 500
    population_size = 400
    neighbor_num = 80

    position_range = []
    for i in range(30):
        position_range.append([0.0, 1.0])
        
    moead = MOEAD(2, 30, zdt1_func)
    moead.initPopulation(population_size, neighbor_num, position_range)
    
    moead.run(generation_number)
    
    paretoFront = getParetoFrontZDT1()
    
    fitnessX = []
    fitnessY = []
    for p in moead.population:
        fitnessX.append(p.fitness[0])
        fitnessY.append(p.fitness[1])
    populationFitness = np.vstack((fitnessX, fitnessY))
    
    VisualizeParetoFront(populationFitness, paretoFront)