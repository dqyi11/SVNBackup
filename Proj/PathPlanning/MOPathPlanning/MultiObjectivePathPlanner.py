'''
Created on Nov 18, 2014

@author: daqing_yi
'''
import numpy as np

class MultiObjectivePathPlanner(object):
    
    def __init__(self, fitnessMgr, start, end, sample_num):
        self.fitnessMgr = fitnessMgr
        self.start = start
        self.end = end
        
        self.obj_num = self.fitnessMgr.fitnessNum + 1
        self.solution_dim = sample_num * 2
        
    def getDistance(self, fromPos, toPos):
        return np.sqrt((fromPos[0]-toPos[0])**2+(fromPos[1]-toPos[1])**2)
        
    def getFitness(self, solution):
        fitness = np.zeros(self.obj_num)
        dist = 0.0
        dist += self.getDistance(self.start, solution[0])
        for i in range(1,self.solution_dim-1):
            dist += self.getDistance(solution[i], solution[i+1])
        dist += self.getDistance(solution[self.solution_dim-1], end)
        
        fitness[0] = dist
        for j in range(1, self.obj_num):
            for s in solution:
                fitness[j] += self.fitnessMgr.getFitnessValue(j, s[0], s[1])
            
            
        
        
        
        