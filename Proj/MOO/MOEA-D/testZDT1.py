'''
Created on Oct 18, 2014

@author: daqing_yi
'''

from zdt import *
from moead import *
from Visualization import *

if __name__ == '__main__':
    
    generation_number = 400
    population_size = 100
    neighbor_num = 30

    position_range = getPositionRangeZDT1()
        
    moead = MOEAD(2, 30, zdt1_func)
    moead.initPopulation(population_size, neighbor_num, position_range)
    
    paretoFront = getParetoFrontZDT1()
    #moead.run(generation_number)
    for t in range(generation_number):
        print "@Generation  " + str(t)
        moead.evolve()
        
        if t%100 == 0:
            fitnessX = []
            fitnessY = []
            for p in moead.population:
                fitnessX.append(p.fitness[0])
                fitnessY.append(p.fitness[1])
            populationFitness = np.vstack((fitnessX, fitnessY))
        
            VisualizeParetoFront(populationFitness, paretoFront, "MOEA-D ZDT1", False)