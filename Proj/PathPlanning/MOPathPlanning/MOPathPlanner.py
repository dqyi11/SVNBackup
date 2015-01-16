'''
Created on Jan 15, 2015

@author: daqing_yi
'''

import numpy as np
from nsga2 import *

class MOPathPlanner(object):
    
    def __init__(self, costFuncs, start, end, sample_num):
        self.costFuncs = costFuncs
        self.start = start
        self.end = end
        
        self.obj_num = len(self.costFuncs)
        self.solution_dim = sample_num * 2
        self.sample_num = sample_num
        
    def getDistance(self, fromPos, toPos):
        return np.sqrt((fromPos[0]-toPos[0])**2+(fromPos[1]-toPos[1])**2)
        
    def getFitness(self, solution):
        fitness = np.zeros(self.obj_num)
        
        for j in range(self.obj_num):
            fitness[j] += self.costFuncs[j]([self.start[0], self.start[1]], [solution[0], solution[1]])
            for i in range(0, self.solution_dim-3, 2):
                #fitness[j] += self.fitnessMgr.getFitnessValue(j-1, solution[i], solution[i+1])
                fitness[j] += self.costFuncs[j]([solution[i],solution[i+1]], [solution[i+2], solution[i+3]])
            fitness[j] += self.costFuncs[j]([solution[self.solution_dim-2],solution[self.solution_dim-1]], [self.end[0], self.end[1]])
        return fitness
    
    def findSolutions(self, population_num, generation_num, position_range):
        self.nsga = NSGAII(self.obj_num, self.solution_dim, self.getFitness)
        self.nsga.initPopulation(population_num, position_range)
        for i in range(generation_num):
            print "iteration " + str(i)
            self.nsga.evolve()
        solutions = []
        for i in range(population_num):
            solutions.append(self.convertSolutionToPath(self.nsga.population[i]))
        return solutions
        
    def convertSolutionToPath(self, solution):
        path = []
        path.append([self.start[0], self.start[1]])
        for i in range(self.sample_num):
            path.append([solution.position[i*2], solution.position[i*2+1]])
        path.append([self.end[0], self.end[1]])
        return path