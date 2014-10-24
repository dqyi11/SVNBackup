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

class MOEAD(object):

    def __init__(self, objective_num, solution_dim,  fitness_func, crossover_rate=1.0, mutation_rate=1.0, mutation_var = 0.1):
        self.objective_num = objective_num
        self.solution_dim = solution_dim
        self.fitness_func = fitness_func
        self.mutation_rate = mutation_rate
        self.mutation_var = mutation_var
        self.crossover_rate = crossover_rate
        self.F = 0.5
        self.CR = 0.5
        self.population = []
        self.external_population = []
        self.utopia_fitness = np.zeros(self.objective_num, np.float)
        for k in range(self.objective_num):
            self.utopia_fitness[k] = np.inf
        
    def initPopulation(self, population_size, neighbor_num, position_range ):
        
        self.population_size = population_size
        self.neighbor_num = neighbor_num
        self.position_range = position_range
        
        self.initWeights()
        self.population = []
        
        for i in range(self.population_size):
            p = Solution(self.objective_num, self.solution_dim, self.neighbor_num, self.weights[i])
            rndSeeds = np.random.random(self.solution_dim)
            for k in range(self.solution_dim):
                p.position[k] = rndSeeds[k] * (position_range[k][1]-position_range[k][0]) + position_range[k][0] 
            self.population.append(p)
            
        self.initNeighborhood()
        self.updateFitness()
        
    def run(self, generation_num):
           
        for t in range(generation_num):
            print "@Generation  " + str(t)
            
            self.evolve()
            
    def evolve(self):
        
        for i in range(self.population_size):
        
            #print "generate new population"
            np = self.geneticOperation(i, self.F)
            
            #print "update fitness and the utopia position"
            np.fitness = self.fitness_func(np.position)
            
            for k in range(self.objective_num):
                if np.fitness[k] < self.utopia_fitness[k]:
                    self.utopia_fitness[k] = np.fitness[k]
            
            #print "update the neighboring solutions"
            p = self.population[i]
            for j in range(p.neighbor_num):
                q_idx = p.neighbor_indices[j]
                q = self.population[q_idx]
                np_val = self.calcSubObjective(np.fitness, p.subproblem_weight)
                q_val = self.calcSubObjective(q.fitness, p.subproblem_weight)
                if np_val <= q_val:
                    q.position = np.position
                    q.fitness = self.fitness_func(q.position)
            
            '''
            #print "update EP"            
            # update EP (external population)
            for i in range(self.population_size):
                p = self.population[i]
                nondominance = True
                for ep in self.external_population:
                    if p >> ep:
                        self.external_population.remove(ep)
                    if ep >> p:
                        nondominance = False
                if nondominance == True:
                    self.external_population.append(p)
            '''

        
    def initWeights(self):
        
        self.weights = []
        for i in range(self.population_size):
            weight = np.zeros(self.objective_num, np.float)
            if self.objective_num == 2:
                weight[0] = float(i) / self.population_size
                weight[1] = float(self.population_size - i) / self.population_size
            self.weights.append(weight)
            
    
    def initNeighborhood(self):
        
        for i in range(self.population_size):
            p = self.population[i]
            dist = np.zeros(self.population_size, np.float)
            for j  in range(self.population_size):
                if i != j:
                    dist[i] = np.linalg.norm(self.weights[i] - self.weights[j], 2)
            
            sorted_idx = np.argsort(dist)    
            p.neighbor_indices = sorted_idx[:self.neighbor_num]
            

    def updateFitness(self):
        
        fitness_list = []
        for i in range(self.population_size):
            p = self.population[i]    
            p.fitness = self.fitness_func(p.position)
            fitness_list.append(p.fitness)
        
        self.utopia_fitness = np.array(fitness_list).argmin(0)
        #print self.utopia_fitness            

    
    def geneticOperation(self, idx, F):
        #generate a new individual from a subproblem and its neighbors
        
        p = self.population[idx]
        
        nb1_idx = idx
        nb2_idx = idx
        nb3_idx = idx
        while nb1_idx == idx:
            nb1_idx = np.random.choice(p.neighbor_indices)
        while nb2_idx == idx or nb2_idx == nb1_idx:
            nb2_idx = np.random.choice(p.neighbor_indices)
        while nb3_idx == idx or nb3_idx == nb2_idx or nb3_idx == nb1_idx:
            nb3_idx = np.random.choice(p.neighbor_indices)

        nb1_pos = self.population[nb1_idx].position
        nb2_pos = self.population[nb2_idx].position
        nb3_pos = self.population[nb3_idx].position
        newPos = nb1_pos + F * (nb2_pos - nb3_pos)
        
        newSolution = Solution(p.objective_num, p.solution_dim, p.neighbor_num, p.subproblem_weight)
        
        rndVals = np.random.random(p.solution_dim)
        for i in range(p.solution_dim):
            if rndVals[i] < self.crossover_rate:
                newSolution.position[i] = newPos[i]
            else:
                newSolution.position[i] = p.position[i]
                
            newSolution.position[i] = np.max([ self.position_range[i][0],  newSolution.position[i] ])
            newSolution.position[i] = np.min([ self.position_range[i][1],  newSolution.position[i] ])

        rndSeeds = np.random.random(p.solution_dim)
        for i in range(p.solution_dim):
            if rndSeeds[i] < self.mutation_rate:
                val_min = self.position_range[i][0]
                val_max = self.position_range[i][1]
                rndVal = np.random.normal(newSolution.position[i], (val_max-val_min)/20)
                new_val = np.min([np.max([rndVal, val_min]), val_max])
                newSolution.position[i] = new_val
                
        return newSolution            
            
            
    def calcSubObjective(self, fitness, weight):
        
        subobj = np.zeros(self.objective_num, np.float)
        for k in range(self.objective_num):
            subobj = weight[k] * np.abs(fitness[k] - self.utopia_fitness[k])
        return subobj.max()
            
            
        
        