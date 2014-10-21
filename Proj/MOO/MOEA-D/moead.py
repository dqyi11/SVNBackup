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
        
    def crossover(self, otherPosition, CR):
        '''
        Crossover operator.
        '''
        child_solution = Solution(self.objective_num, self.solution_dim, self.neighbor_num, self.subproblem_weight)
        
        for i in range(self.solution_dim):
            if np.random.random() < CR:
                child_solution.position[i] = otherPosition[i]
            else:
                child_solution.position[i] = self.position[i]
        
        return child_solution
    
    def mutate(self):
        '''
        Mutation operator.
        '''
        self.position[np.random.randint(self.solution_dim)] = np.random.random()
        
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

    def __init__(self, objective_num, solution_dim,  fitness_func, crossover_rate=0.5, mutation_rate=0.5, mutation_var = 0.1):
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
        
        self.utopia_fitness = np.array(fitness_list).argmin(0)
        #print self.utopia_fitness
            
    def run(self, generation_num):
           
        for t in range(generation_num):
            print "@Generation  " + str(t)
            
            #generate new population
            self.geneticOperation(self.F, self.CR)
            
            #update fitness and the utopia position
            fitness_list = []
            fitness_list.append(self.utopia_fitness)
            for i in range(self.population_size):
                p = self.population[i]
                p.fitness = self.fitness_func(p.position)
                fitness_list.append(p.fitness)
        
            self.utopia_fitness = np.array(fitness_list).argmin(0)
            
            #update the neighboring solutions
            for i in range(self.population_size):
                p = self.population[i]
                for j in range(p.neighbor_num):
                    q_idx = p.neighbor_indices[j]
                    q = self.population[q_idx]
                    p_val = self.calcSubObjective(p.fitness, q.subproblem_weight)
                    q_val = self.calcSubObjective(q.fitness, q.subproblem_weight)
                    if p_val <= q_val:
                        q.position = p.position
                        q.fitness = self.fitness_func(q.position)
                        
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
    
    def geneticOperation(self, F, CR):
        #generate a new individual from a subproblem and its neighbors
        for p in self.population:
            
            neighborIndices = p.neighbor_indices[0:self.neighbor_num]
            
            print "len(neighborIndices) "+str(len(neighborIndices))
            print neighborIndices
            
            p1_idx = np.random.choice(neighborIndices)
            p2_idx = np.random.choice(neighborIndices)
            p3_idx = np.random.choice(neighborIndices)
            p1 = self.population[p1_idx]
            p2 = self.population[p2_idx]
            p3 = self.population[p3_idx]
            
            new_position = p1.position + F * (p2.position - p3.position)
            
            p = p.crossover(new_position, CR)
            
            if np.random.random() < self.mutation_rate:
                p.mutate()     
            
            
    def calcSubObjective(self, fitness, weight):
        
        subobj = np.zeros(self.objective_num, np.float)
        for k in range(self.objective_num):
            subobj = weight[k] * np.abs(fitness[k] - self.utopia_fitness[k])
        return subobj.max()
            
            
        
        