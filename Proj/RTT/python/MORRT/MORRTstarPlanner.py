'''
Created on Jan 5, 2015

@author: daqing_yi
'''

import numpy as np
from MORRTstar import *
from MORRTVisualizer import *

class MORRTstarPlanner(object):

    def __init__(self, planning_range, segment_length, objective_num, cost_funcs=None, subproblem_num, map_file=None):
        self.planningRange = planning_range
        self.segmentLength = segment_length
        self.mapFile = map_file
        self.objectiveNum = objective_num
        self.costFuncs = cost_funcs
        self.subproblemNum = subproblem_num
        
        self.morrts = MORRTstar(self.planningRange, self.segmentLength, self.objectiveNum)
        self.morrts_viz = MORRTVisualizer(self.morrts)
        
        self.weights = []
        for i in range(self.population_size):
            weight = np.zeros(self.objective_num, np.float)
            if self.objectiveNum == 2:
                weight[0] = float(i) / self.subproblemNum
                weight[1] = float(self.population_size - i) / self.population_size
            self.weights.append(weight)

            
    def findPath(self, start, goal, iterationNum):
        
        self.morrts.init(start, goal, self.costFuncs, self.weights, self.subproblemNum)

        for it in range(iterationNum):
            self.morrts.extend()
                
        paths = self.morrts.findPaths()
        
        self.morrts_viz.activePaths = paths
        
        self.rrts_viz.update()
        
        return paths
            
        
        
        
            
        
        