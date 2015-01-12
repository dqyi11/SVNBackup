'''
Created on Jan 5, 2015

@author: daqing_yi
'''

import numpy as np
from MORRTstar import *
from MORRTVisualizer import *

class MORRTstarPlanner(object):

    def __init__(self, planning_range, segment_length, objective_num, cost_funcs, subproblem_num, map_file=None):
        self.planningRange = planning_range
        self.segmentLength = segment_length
        self.mapFile = map_file
        self.objectiveNum = objective_num
        self.costFuncs = cost_funcs
        self.subproblemNum = subproblem_num
        
        self.morrts = MORRTstar(self.planningRange, self.segmentLength, self.objectiveNum, self.subproblemNum)
        self.morrts_viz = MORRTVisualizer(self.morrts)
        
        self.weights = []
        for i in range(self.subproblemNum):
            weight = np.zeros(self.objectiveNum, np.float)
            if self.objectiveNum == 2:
                weight[0] = float(i) / self.subproblemNum
                weight[1] = float(self.subproblemNum - i) / self.subproblemNum
            self.weights.append(weight)
            
        if self.mapFile != None:
            self.mokrrts.loadMap(self.mapFile)

            
    def findPaths(self, start, goal, iterationNum):
        
        self.morrts.init(start, goal, self.costFuncs, self.weights)

        for it in range(iterationNum):
            #print "Iter@" + str(it)
            self.morrts.extend()
            self.morrts_viz.update()
                
        paths = self.morrts.findPaths()
        
        self.morrts_viz.activePaths = paths
        
        self.morrts_viz.update()
        
        return paths
            
        
        
        
            
        
        