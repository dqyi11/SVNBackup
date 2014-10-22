'''
Created on Oct 21, 2014

@author: daqing_yi
'''

from zdt import *
from nsga2 import *
from Visualization import *

if __name__ == '__main__':
    
    generation_number = 10
    population_size = 50

    position_range = []
    for i in range(30):
        position_range.append([0.0, 1.0])
        
    nsga2 = NSGAII(2, 30, zdt6_func)
    nsga2.initPopulation(population_size, position_range)
    
    nsga2.run(generation_number)
    
    paretoFront = getParetoFrontZDT6()
    
    fitnessX = []
    fitnessY = []
    for p in nsga2.population:
        fitnessX.append(p.fitness[0])
        fitnessY.append(p.fitness[1])
    populationFitness = np.vstack((fitnessX, fitnessY))
      
    VisualizeParetoFront(populationFitness, paretoFront, "ZDT4")
    