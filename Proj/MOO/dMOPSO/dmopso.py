'''
Created on Oct 18, 2014

@author: daqing_yi
'''

import numpy as np
import random, copy

class Solution(object):
    
    def __init__(self, objective_num, solution_dim, subproblem_weight):
        self.objective_num = objective_num
        self.solution_dim = solution_dim
        self.subproblem_weight = subproblem_weight
        self.fitness = np.zeros(objective_num, np.float)
        self.position = np.zeros(solution_dim, np.float)
        self.velocity = np.zeros(solution_dim, np.float)
        self.pb_position = np.zeros(solution_dim, np.float)
        self.pb_fitness = np.zeros(objective_num, np.float)
        self.age = 0

class dMOPSO(object):
    
    def __init__(self, objective_num, solution_dim,  fitness_func):
        self.objective_num = objective_num
        self.solution_dim = solution_dim
        self.fitness_func = fitness_func
        self.population = []
        self.gb_set = []
        self.utopia_fitness = np.zeros(self.objective_num, np.float)
        
        
    def setParameters(self, chi, phi_p, phi_g, age_threshold, gamma):
        self.chi = chi
        self.phi_p = phi_p
        self.phi_g = phi_g
        self.age_threshold = age_threshold
        self.gamma = gamma
        
    def initPopulation(self, population_size, position_range):
        
        self.population_size = population_size
        self.position_range = position_range
        self.weights = self.initWeights()
        
        self.population = []
        for i in range(self.population_size):
            p = Solution(self.objective_num, self.solution_dim, self.weights[i])
            rndSeeds = np.random.random(self.solution_dim)
            for k in range(self.solution_dim):
                p.position[k] = rndSeeds[k] * (position_range[k][1]-position_range[k][0]) + position_range[k][0]
                p.fitness = self.fitness_func(p.position)
                p.pb_position = copy.deepcopy(p.position)
                p.pb_fitness = copy.deepcopy(p.fitness)
            self.population.append(p)
            self.gb_set.append(copy.deepcopy(p))
        
        
            
    def run(self, generation_num):
  
        for t in range(generation_num):
            print "@Generation  " + str(t)
            self.evolve()


    def evolve(self):
            
        random.shuffle(self.gb_set)
            
        # update position
        for i in range(self.population_size):
            p = self.population[i]
            if p.age < self.age_threshold:
                rndPbVal = np.random.random(self.solution_dim)
                rndGbVal = np.random.random(self.solution_dim)
                for d in range(self.solution_dim):
                    p.velocity[d] = self.chi * p.velocity[d] + self.phi_p * rndPbVal[d] * (p.pb_position[d] - p.position[d]) + self.phi_g * rndGbVal[d] * (self.gb_set[i].position[d] - p.position[d])
                    p.position[d] = p.position[d] + p.velocity[d]
            else:
                p.velocity = np.zeros(self.solution_dim, np.float)
                p.age = 0
                # reinitialize position
                for d in range(self.solution_dim):
                    norm_mean = (self.gb_set[i].position[d]-p.pb_position[d])/2.0
                    norm_var = np.abs(self.gb_set[i].position[d]-p.pb_position[d])
                    if norm_var == 0:
                        norm_var = 0.00000001
                    p.position[d] = np.random.normal(loc=norm_mean, scale=norm_var)
                

            # repair bounds
            for d in range(self.solution_dim):
                if p.position[d] > self.position_range[d][1]:
                    p.position[d] = self.position_range[d][1]
                    p.velocity[d] = - self.gamma * p.velocity[d]
                elif p.position[d] < self.position_range[d][0]:
                    p.position[d] = self.position_range[d][0]
                    p.velocity[d] = - self.gamma * p.velocity[d]
    
        # update fitness
        for p in self.population:
            p.fitness = self.fitness_func(p.position)
            for k in range(self.objective_num):
                if p.fitness[k] < self.utopia_fitness[k]:
                    self.utopia_fitness[k] = p.fitness[k]
                    
        # update personal best 
        for p in self.population:
            if self.calcSubObjective(p.position, p.subproblem_weight) < self.calcSubObjective(p.pb_position, p.subproblem_weight):
                p.pb_position = copy.deepcopy(p.position)
                p.age = 0
            else:
                p.age += 1
                
        # update global best
        P = copy.deepcopy(self.gb_set) 
        currentPopulation = copy.deepcopy(self.population)
        P.extend(currentPopulation)
        
        self.gb_set = self.updateGlobalBest(self.weights, P)  
                
            
        
    def initWeights(self):
        weights = []
        for i in range(self.population_size):
            weight = np.zeros(self.objective_num, np.float)
            if self.objective_num == 2:
                weight[0] = float(i) / float(self.population_size)
                weight[1] = float(self.population_size - i) / float(self.population_size)
            weights.append(weight)
        
        return weights
    
    def calcSubObjective(self, position, weight):
        
        subobj = np.zeros(self.objective_num, np.float)
        fitness = self.fitness_func(position)
        for k in range(self.objective_num):
            subobj[k] = weight[k] * np.abs(fitness[k] - self.utopia_fitness[k])
        return subobj.max()
    
    def updateGlobalBest(self, W, P):
        
        gb_set = []
        
        for w in W:
            subobj_vals = []
            for p in P:
                subobj_vals.append(self.calcSubObjective(p.position, w))
            
            min_idx = np.argmin(subobj_vals)
            p = P[min_idx]
            gb_set.append(p)
            P.remove(p)    
        
        return gb_set
            