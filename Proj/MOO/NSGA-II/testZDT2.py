'''
Created on Oct 21, 2014

@author: daqing_yi
'''

from zdt import *
from nsga2 import *
from Visualization import *

if __name__ == '__main__':
    
    generation_number = 200
    population_size = 100

    position_range = []
    for i in range(30):
        position_range.append([0.0, 1.0])
        
    nsga2 = NSGAII(2, 30, zdt2_func)
    nsga2.initPopulation(population_size, position_range)
    
    nsga2.run(generation_number)
    
    paretoFront = getParetoFrontZDT2()
    
    fitnessX = []
    fitnessY = []
    for p in nsga2.population:
        fitnessX.append(p.fitness[0])
        fitnessY.append(p.fitness[1])
    populationFitness = np.vstack((fitnessX, fitnessY))
      
    VisualizeParetoFront(populationFitness, paretoFront, "NSGA-II ZDT2")
    