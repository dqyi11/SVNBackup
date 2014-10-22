'''
Created on Oct 18, 2014

@author: daqing_yi
'''


from zdt import *
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
    
    generation_number = 100
    population_size = 1000
        
    dmopso = dMOPSO(2, 30, zdt4_func)
    dmopso.setParameters(chi, phi_p, phi_g, age_threshold, gamma)
    dmopso.initPopulation(population_size, position_range)

    
    dmopso.run(generation_number)
    
    paretoFront = getParetoFrontZDT4()
    
    fitnessX = []
    fitnessY = []
    for p in dmopso.population:
        fitnessX.append(p.fitness[0])
        fitnessY.append(p.fitness[1])
    populationFitness = np.vstack((fitnessX, fitnessY))
    
    VisualizeParetoFront(populationFitness, paretoFront)    
    