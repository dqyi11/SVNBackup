'''
Created on Nov 15, 2014

@author: daqing_yi
'''

import numpy as np
import sys, copy

class Solution(object):
    
    def __init__(self, objective_num, solution_dim, mutate_var):
        self.objective_num = objective_num
        self.solution_dim = solution_dim
        self.mutate_var = mutate_var
        self.fitness = np.zeros(objective_num, np.float)
        self.position = np.zeros(solution_dim, np.float)
        self.rank = sys.maxint
        self.distance = 0.0
        
        self.fraction = 2.0 / self.solution_dim
        
    def crossover(self, other, data_range):
        '''
        Crossover operator.
        '''
        pass
    
    def __eq__(self, other):
        for i in range(self.solution_dim):
            if self.position[i] != other.position[i]:
                return False
        return True
    
    def mutate(self, data_range):
        '''
        Mutation operator.
        '''
        pass
    
    def __rshift__(self, other):
        '''
        True if this solution dominates the other (">>" operator).
        '''
        dominates = False        
        for i in range(self.objective_num):
            if self.fitness[i] > other.fitness[i]:
                return False
            elif self.fitness[i] < other.fitness[i]:
                dominates = True
        return dominates
    
    def __lshift__(self, other):
        '''
        True if this solution is dominated by the other ("<<" operator).
        '''
        return other >> self
    
    def compareCrowding(self, other):
        '''
        Compare the two solutions based on crowded comparison.
        '''
        if self.rank < other.rank:
            return 1
            
        if self.rank > other.rank:
            return -1
            
        if self.distance > other.distance:
            return 1
            
        if self.distance < other.distance:
            return -1
            
        return 0

class NSGAIII(object):

    def __init__(self, objective_num, solution_dim,  fitness_func, crossover_rate=1.0, mutation_rate=0.1, mutation_var=0.1):

        self.objective_num = objective_num
        self.solution_dim = solution_dim
        self.fitness_func = fitness_func
        self.crossover_rate = crossover_rate
        self.mutation_rate = mutation_rate
        self.mutation_var = mutation_var
        self.population = []
        
    def initPopulation(self, population_size, position_range):
        
        self.population_size = population_size
        self.position_range = position_range
        
        self.population = []
        for i in range(self.population_size):
            p = Solution(self.objective_num, self.solution_dim, self.mutation_var)
            rndSeeds = np.random.random(self.solution_dim)
            for k in range(self.solution_dim):
                p.position[k] = rndSeeds[k] * (position_range[k][1]-position_range[k][0]) + position_range[k][0] 
                p.fitness = self.fitness_func(p.position)
            self.population.append(p)
            
    def run(self, generation_num):
        for i in range(generation_num):
            print "@Generation  " + str(i) 
            self.evolve()
        
        
    def evolve(self):
        pass