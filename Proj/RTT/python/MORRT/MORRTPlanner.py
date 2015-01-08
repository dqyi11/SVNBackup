'''
Created on Jan 5, 2015

@author: daqing_yi
'''

import numpy as np
from RRTstar import *

class MORRTPlanner(object):

    def __init__(self, planning_range, segment_length, objective_num, cost_funcs=None, map_file=None):
        self.planningRange = planning_range
        self.segmentLength = segment_length
        self.mapFile = map_file
        self.objectiveNum = objective_num
        self.costFuncs = cost_funcs
        
    def initSubproblem(self, subproblem_num):
        
        self.refRRTSet = []
        for k in range(self.objectiveNum):
            rrt = RRTstar(self.planningRange, self.segmentLength, 1)
            self.refRRTSet.append(rrt)
        
        self.subproblemNum = subproblem_num
        
        self.weights = []
        for i in range(self.population_size):
            weight = np.zeros(self.objective_num, np.float)
            if self.objectiveNum == 2:
                weight[0] = float(i) / self.subproblemNum
                weight[1] = float(self.population_size - i) / self.population_size
            self.weights.append(weight)
            
        self.rrtSet = []
        for i in range(self.subproblemNum):
            rrt = RRTstar(self.planningRange, self.segmentLength, self.objective_num)
            self.rrtSet.append(rrt)
            
    def findPath(self, start, goal, iterationNum):
        
        for k in range(self.objectiveNum):
            self.refRRTSet[k].init(start, goal, self.costFuncs[k], [1.0])
            
        for i in range(self.subproblemNum):
            self.rrtSet[i].init(start, goal, self.costFuncs, self.weights[i])
            
        for it in range(iterationNum):
            for i in range(self.subproblemNum):
                self.rrtSet[i].extend()
                
        path = self.rrts.findPath()
        
        self.rrts_viz.activePath = path
        
        self.rrts_viz.update()
        
        return path
            
        
        
        
            
        
        