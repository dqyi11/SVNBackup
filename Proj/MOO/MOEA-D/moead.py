'''
Created on Oct 18, 2014

@author: daqing_yi
'''

import numpy as np

class Solution(object):
    
    def __init__(self, objective_num, solution_dim, subproblem_weight):
        self.objective_num = objective_num
        self.solution_dim = solution_dim
        self.fitness = np.zeros(objective_num, np.float)
        self.position = np.zeros(solution_dim, np.float)
        self.subproblem_weight = subproblem_weight

class MOEAD(object):

    def __init__(self, objective_num, solution_dim,  fitness_func):
        self.objective_num = objective_num
        self.solution_dim = solution_dim
        self.fitness_func = fitness_func
        self.population = []
        
    def initPopulation(self, population_size, position_range):
        
        self.population_size = population_size
        self.range = range
        rndSeeds = np.random.random(self.population_size*self.solution_dim)
        weights = self.initWeights()
        self.population = []
        for i in range(self.population_size):
            p = Solution(self.objective_num, self.solution_dim, weights[i])
            for k in range(self.solution_dim):
                p.position[k] = rndSeeds[k+i*self.solution_dim] * (position_range[k][1]-position_range[k][0]) + position_range[k][0] 
            self.population.append(p)
            
    def run(self, generation_num):
           
        # update fitness
        for p in self.population:
            p.fitness = self.fitness_func(p.position)
            
        
        for i in range(generation_num):
            print "@Generation  " + str(i)
            continue
        
        
    def initWeights(self):
        weights = []
        for i in range(self.population_size):
            weight = np.zeros(self.objective_num, np.float)
            if self.objective_num == 2:
                weight[0] = float(i) / float(self.population_size)
                weight[1] = float(self.population_size - i) / float(self.population_size)
            weights.append(weight)
            
        distanceMatrix = np.zeros((self.population_size, self.population_size), np.float)
        for i in range(self.population_size):
            for j  in range(1, self.population_size, 1):
                dist = np.linalg.norm(self.population[i].position - self.population[j].position, 2)
                distanceMatrix[i, j] = dist
                distanceMatrix[j, i] = dist
        return weights
        
        