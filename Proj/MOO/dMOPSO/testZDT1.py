'''
Created on Oct 18, 2014

@author: daqing_yi
'''

from zdt import *
from dmopso import *
from Visualization import *

if __name__ == '__main__':

    position_range = getPositionRangeZDT1()
        
    chi = 0.8
    phi_p = 1.5
    phi_g = 1.5
    age_threshold = 3
    gamma = 3
    
    generation_number = 100
    population_size = 200
        
    dmopso = dMOPSO(2, 30, zdt1_func)
    dmopso.setParameters(chi, phi_p, phi_g, age_threshold, gamma)
    dmopso.initPopulation(population_size, position_range)

    paretoFront = getParetoFrontZDT1()
    
    #dmopso.run(generation_number)
    
    for i in range(generation_number):
        print "Iteration " + str(i)
        dmopso.evolve()
        
        if i%20 == 0:
    
            fitnessX = []
            fitnessY = []
            for p in dmopso.population:
                fitnessX.append(p.fitness[0])
                fitnessY.append(p.fitness[1])
            populationFitness = np.vstack((fitnessX, fitnessY))
            
            VisualizeParetoFront(populationFitness, paretoFront, "dMOPSO ZDT1", False) 
    