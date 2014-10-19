'''
Created on Oct 18, 2014

@author: daqing_yi
'''

import numpy as np

class Solution(object):
    
    def __init__(self, objective_num, solution_dim):
        self.objective_num = objective_num
        self.solution_dim = solution_dim
        self.fitness = np.zeros(objective_num, np.float)
        self.position = np.zeros(solution_dim, np.float)
        
        self.rank = np.iinfo(np.int).max
        self.distance = 0.0
        
    def crossover(self, other):
        '''
        Crossover operator.
        '''
        child_solution = Solution(self.objective_num, self.solution_dim)
        
        for i in range(self.solution_dim):
            child_solution.position[i] = np.sqrt(self.position[i] * other.position[i])
        
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

class NSGAII(object):

    def __init__(self, objective_num, solution_dim,  fitness_func, mutation_rate=0.1, crossover_rate=1.0):

        self.objective_num = objective_num
        self.solution_dim = solution_dim
        self.mutation_rate = mutation_rate
        self.crossover_rate = crossover_rate
        self.fitness_func = fitness_func
        self.population = []
        
    def initPopulation(self, population_size, position_range):
        
        self.population_size = population_size
        self.range = range
        rndSeeds = np.random.random(self.population_size*self.solution_dim)
        self.population = []
        for i in range(self.population_size):
            p = Solution(self.objective_num, self.solution_dim)
            for k in range(self.solution_dim):
                p.position[k] = rndSeeds[k+i*self.solution_dim] * (position_range[k][1]-position_range[k][0]) + position_range[k][0] 
            self.population.append(p)
            
    def run(self, generation_num):
        
        # update fitness
        for p in self.population:
            p.fitness = self.fitness_func(p.position)
            
        Q = []
        P = self.population
        
        for i in range(generation_num):
            print "@Generation  " + str(i)
             
            R = []
            R.extend(P)
            R.extend(Q)
            
            fronts = self.fastNondominatedSort(R)
            
            del P[:]
            
            for front in fronts:
                if len(front) == 0:
                    break
                
                self.crowdingDistanceAssignment(front);
                P.extend(front)
                
                if len(P) >= self.population_size:
                    break
            
            self.sortByCrowding(P)
            
            if len(P) > self.population_size:
                del P[self.population_size:]
                
            Q = self.makeNewPop(P)
            
        self.population = P
            
    def fastNondominatedSort(self, P):
        
        fronts = []
        
        S = {}
        n = {}
        
        front = []
        fronts.append(front)    
        
        for p in P:
            S[p] = []
            n[p] = 0
          
            for q in P:
                if p == q:
                    continue
                
                if p >> q:
                    S[p].append(q)
                elif q >> p:
                    n[p] += 1
                    
                if n[p] == 0:
                    front.append(p)
        
        i = 0
        while len(fronts[i]) != 0:
            next_front = []
            for p in fronts[i]:
                for q in S[p]:
                    n[q] -= 1
                    if n[q] == 0:
                        next_front.append(q)
            i +=1
            fronts.append(next_front)
            
        return fronts
            
    def crowdingDistanceAssignment(self, front):
        '''
        Assign a crowding distance for each solution in the front. 
        '''
        for p in front:
            p.distance = 0
        for k in range(self.objective_num):
            self.sortByObjective(front, k)
            
            front[0].distance = float('inf')
            front[len(front) - 1].distance = float('inf')
            
            for i in range(1, len(front) - 1):
                front[i].distance += (front[i + 1].distance - front[i - 1].distance)
                
    def sortByObjective(self, P, obj_idx):
        
        for i in range(len(P)-1, -1, -1):
            for j in range(1, i + 1):
                s1 = P[j - 1]
                s2 = P[j]
                
                if s1.fitness[obj_idx] > s2.fitness[obj_idx]:
                    P[j - 1] = s2
                    P[j] = s1
                    
    def sortByCrowding(self, P):
        for i in range(len(P) - 1, -1, -1):
            for j in range(1, i + 1):
                s1 = P[j - 1]
                s2 = P[j]
                
                if s1.compareCrowding(s2) < 0:
                    P[j - 1] = s2
                    P[j] = s1
                                
                    
    def makeNewPop(self, P):
        '''
        Make new population Q, offspring of P. 
        '''
        Q = []
               
        while len(Q) != len(P):
            selected_solutions = [None, None]
            
            while selected_solutions[0] == selected_solutions[1]:
                for i in range(2):
                    s1 = np.random.choice(P)
                    s2 = s1
                    while s1 == s2:
                        s2 = np.random.choice(P)
                    
                    if s1.compareCrowding(s2) > 0:
                        selected_solutions[i] = s1
                        
                    else:
                        selected_solutions[i] = s2
            
            if np.random.random() < self.crossover_rate:
                child_solution = selected_solutions[0].crossover(selected_solutions[1])
                
                if np.random.random() < self.mutation_rate:
                    child_solution.mutate()
                    
                child_solution.fitness = self.fitness_func(child_solution.position)
                
                Q.append(child_solution)

        
        return Q
        
                        
                    
                
        
        
        
        
    
        