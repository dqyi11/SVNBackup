'''
Created on Jan 4, 2015

@author: daqing_yi
'''

from Planner import *
from kRRTstar import *
from RRTVisualizer import *

class kRRTstarPlanner(Planner):
    
    def __init__(self, planning_range, segment_length, cost_func=None, map_file=None):
        super(kRRTstarPlanner, self).__init__(planning_range, segment_length, map_file)
        self.krrts = kRRTstar(self.planningRange, self.segmentLength)
        if self.mapFile!= None:
            self.krrts.loadMap(self.mapFile)
        self.krrts_viz = RRTVisualizer(self.krrts)
        self.cost_func = cost_func
        
    def findPath(self, start, goal, iterationNum):
        
        self.krrts.init(start, goal, self.cost_func)
        for i in range(iterationNum):
            self.krrts.extend()
            self.krrts_viz.update()
            
        path = self.krrts.findPath()
        
        self.krrts_viz.activePath = path
        
        self.krrts_viz.update()
        
        return path