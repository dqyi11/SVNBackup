'''
Created on Oct 18, 2014

@author: daqing_yi
'''

from zdt import *
from nsga3 import *
from Visualization import *

if __name__ == '__main__':
    
    generation_number = 100
    population_size = 100

    position_range = getPositionRangeZDT1()
        
    nsga2 = NSGAIII(2, 30, zdt1_func)
    nsga2.initPopulation(population_size, position_range)
    
    paretoFront = getParetoFrontZDT1()
    
    #nsga2.run(generation_number)
    for i in range(generation_number):
        print "iteration " + str(i)
        nsga2.evolve()
    
    fitnessX = []
    fitnessY = []
    for p in nsga2.population:
        fitnessX.append(p.fitness[0])
        fitnessY.append(p.fitness[1])
    populationFitness = np.vstack((fitnessX, fitnessY))
      
    VisualizeParetoFront(populationFitness, paretoFront, "NSGA-II ZDT1", False)
    