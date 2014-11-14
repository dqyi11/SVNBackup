'''
Created on Oct 18, 2014

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
        # Intermediate crossover
        child_1 = Solution(self.objective_num, self.solution_dim, self.mutate_var)
        child_2 = Solution(self.objective_num, self.solution_dim, self.mutate_var)
        rndVals = np.random.random(self.solution_dim)
        crossFlags = np.random.random(self.solution_dim) < self.fraction
        crossFlagIdx = 1 * crossFlags
        for i in range(self.solution_dim):
            child_1.position[i] = self.position[i] + crossFlagIdx[i] * rndVals[i] * 1.2 * (self.position[i] - other.position[i])
            child_2.position[i] = other.position[i] - crossFlagIdx[i] * rndVals[i] * 1.2 * (self.position[i] - other.position[i])
            
            child_1.position[i] = np.max([ data_range[i][0],  child_1.position[i] ])
            child_1.position[i] = np.min([ data_range[i][1],  child_1.position[i] ])
            child_2.position[i] = np.max([ data_range[i][0],  child_2.position[i] ])
            child_2.position[i] = np.min([ data_range[i][1],  child_2.position[i] ])
        return child_1, child_2
    
    def __eq__(self, other):
        for i in range(self.solution_dim):
            if self.position[i] != other.position[i]:
                return False
        return True
    
    def mutate(self, data_range):
        '''
        Mutation operator.
        '''
        rndVals = np.random.random(self.solution_dim)
        drange = np.array(data_range)
        scales = drange[:,1] - drange[:,0]
        for i in range(self.solution_dim):
            if rndVals[i] < self.fraction:
                self.position[i] = self.position[i] + scales[i] * np.random.normal()
                
                self.position[i] = np.max([ data_range[i][0],  self.position[i] ])
                self.position[i] = np.min([ data_range[i][1],  self.position[i] ])
                
        
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
        
        P = self.population
        Q = self.makeNewPop(P)

        R = []
        R.extend(P)
        R.extend(Q)
        
        #print "fast non dominated sort"
        fronts = self.fastNondominatedSort(R)
        
        '''
        total_num = 0
        for f in fronts.values():
            total_num += len(f)
        print "total num " + str(total_num)
        '''
        
        del P[:]
        
        #print "crowding distance assignment"
        for front in fronts.values():
            if len(front) == 0:
                break
            
            self.crowdingDistanceAssignment(front);
            P.extend(front)
            
            if len(P) >= self.population_size:
                break
        
        #print "sort by crowding"
        self.sortByCrowding(P)
        
        #print "Before P Size " + str(len(P))
        
        if len(P) > self.population_size:
            del P[self.population_size:]
            
        #print "After P Size " + str(len(P))
            
        self.population = P
            
    def fastNondominatedSort(self, P):
        
        fronts = {}
        
        S = {}
        n = {}
        
        for p in P:
            # inital the set that contains all the individuals dominted by p
            S[p] = []
            # number of the individuals dominat p
            n[p] = 0
            
        fronts[1] = [] 
          
        for p in P:
            for q in P:
                if p == q:
                    continue
                
                if p >> q:
                    S[p].append(q)
                elif q >> p:
                    n[p] += 1
                    
            if n[p] == 0:
                p.rank = 1
                fronts[1].append(p)
                
        # initial front counter to 1
        i = 1
        while len(fronts[i])!=0: # ith front is not empty
            next_front = []
            for p in fronts[i]:
                for q in S[p]:
                    n[q] -= 1
                    if n[q] == 0:
                        q.rank = i+1
                        next_front.append(q)
            i +=1
            fronts[i] = next_front
            
        return fronts
            
    def crowdingDistanceAssignment(self, front):
        '''
        Assign a crowding distance for each solution in the front. 
        '''
        for p in front:
            p.distance = 0
            
        f_max = np.zeros(self.objective_num, np.float)
        f_min = np.ones(self.objective_num, np.float) * np.inf
        for p in front:
            for k in range(self.objective_num):
                if p.fitness[k] > f_max[k]:
                    f_max[k] = p.fitness[k]
                if p.fitness[k] < f_min[k]:
                    f_min[k] = p.fitness[k]            
            
        for k in range(self.objective_num):
            self.sortByObjective(front, k)
            
            front[0].distance = np.inf
            front[len(front) - 1].distance = np.inf
            
            for i in range(1, len(front) - 1):
                front[i].distance += (front[i + 1].fitness[k] - front[i - 1].fitness[k])/(f_max[k] - f_min[k])
                
    def sortByObjective(self, P, obj_idx):
        for i in range(len(P)-1, -1, -1):
            for j in range(1, i + 1): 
                s1 = P[j-1]
                s2 = P[j]
                if s1.fitness[obj_idx] > s2.fitness[obj_idx]:
                    P[j-1] = s2
                    P[j] = s1
                    
    def sortByCrowding(self, P):
        for i in range(len(P) - 1, -1, -1):
            for j in range(1, i + 1):
                s1 = P[j-1]
                s2 = P[j]
                if P[j - 1].compareCrowding(P[j]) < 0:
                    P[j-1] = s2
                    P[j] = s1
                    
    def makeNewPop(self, P):
        '''
        Make new population Q, offspring of P. 
        '''
        Q = []
        P_size = len(P)
               
        
        while len(Q) <= P_size:
            tournament_selection = [None, None] 
            
            # select
            while tournament_selection[0] == tournament_selection[1]:
                for i in range(2):
                    s1 = np.random.choice(P)
                    s2 = s1
                    while s1 == s2:
                        s2 = np.random.choice(P)
                    
                    if s1.compareCrowding(s2) > 0:
                        tournament_selection[i] = s1
                    else:
                        tournament_selection[i] = s2
                      
            # crossover
            if np.random.random() < self.crossover_rate:
                child_solution = tournament_selection[0].crossover(tournament_selection[1], self.position_range)[0]
                    
                # mutate
                if np.random.random() < self.mutation_rate:
                    child_solution.mutate(self.position_range)
                    
                    # update fitness
                    child_solution.fitness =self.fitness_func(child_solution.position)

                    
                    Q.append(child_solution)

        return Q
        
                        
                    
                
        
        
        
        
    
        