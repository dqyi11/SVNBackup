'''
Created on Oct 18, 2014

@author: daqing_yi
'''

import numpy as np

class Solution(object):
    
    def __init__(self, objective_num, solution_dim, neighbor_num, subproblem_weight):
        self.objective_num = objective_num
        self.solution_dim = solution_dim
        self.fitness = np.zeros(objective_num, np.float)
        self.position = np.zeros(solution_dim, np.float)
        self.neighbor_num = neighbor_num
        self.subproblem_weight = subproblem_weight

class MOEAD(object):

    def __init__(self, objective_num, solution_dim,  fitness_func):
        self.objective_num = objective_num
        self.solution_dim = solution_dim
        self.fitness_func = fitness_func
        self.population = []
        self.utopia_position = np.zeros(self.objective_num, np.float)
        for k in range(self.objective_num):
            self.utopia_position[k] = np.inf
        
    def initPopulation(self, position_range, population_size=100, neighbor_num=30 ):
        
        self.population_size = population_size
        self.neighbor_num = neighbor_num
        self.position_range = position_range
        
        weights, weight_distances = self.initWeights()
        self.population = []
        
        fitness_list = []
        for i in range(self.population_size):
            p = Solution(self.objective_num, self.solution_dim, self.neighbor_num, weights[i])
            rndSeeds = np.random.random(self.solution_dim)
            for k in range(self.solution_dim):
                p.position[k] = rndSeeds[k] * (position_range[k][1]-position_range[k][0]) + position_range[k][0] 
            self.population.append(p)
            p.neighbor_indices = np.argsort(weight_distances[i])
            p.fitness = self.fitness_func(p.position)
            fitness_list.append(p.fitness)
        
        self.utopia_position = np.array(fitness_list).argmin(0)
        #print self.utopia_position
            
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
            
        weight_distances = []
        for i in range(self.population_size):
            weight_distances.append(np.zeros(self.population_size,np.float))
        for i in range(self.population_size):
            for j  in range(1, self.population_size, 1):
                dist = np.linalg.norm(weights[i] - weights[j], 2)
                weight_distances[i][j] = dist
                weight_distances[j][i] = dist
        return weights, weight_distances
        
        