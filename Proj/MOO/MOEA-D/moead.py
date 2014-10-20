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

    def __init__(self, objective_num, solution_dim,  fitness_func, mutation_rate=0.5, crossover_rate=0.5):
        self.objective_num = objective_num
        self.solution_dim = solution_dim
        self.fitness_func = fitness_func
        self.mutation_rate = mutation_rate
        self.crossover_rate = crossover_rate
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
           
        for i in range(generation_num):
            print "@Generation  " + str(i)
            
            #generate new population
            self.geneticOperation()
            
            #update fitness and the utopia position
            fitness_list = []
            fitness_list.append(self.utopia_position)
            for i in range(self.population_size):
                p = self.population[i]
                p.fitness = self.fitness_func(p.position)
                fitness_list.append(p.fitness)
        
            self.utopia_position = np.array(fitness_list).argmin(0)
            
            #update the neighbors
            
            
        
        
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
    
    def geneticOperation(self):
        #generate a new individual from a subproblem and its neighbors
        for p in self.population:
            
            neighborIndices = p.neighbor_indices[0:self.neighbor_num]
            
            print "len(neighborIndices) "+str(len(neighborIndices))
            print neighborIndices
            
            p1_idx = np.random.choice(neighborIndices)
            p2_idx = np.random.choice(neighborIndices)
            
            p1 = self.population[p1_idx]
            p2 = self.population[p2_idx]
            
            
        
        