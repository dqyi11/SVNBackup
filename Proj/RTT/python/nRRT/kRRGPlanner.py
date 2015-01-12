'''
Created on Jan 4, 2015

@author: daqing_yi
'''

from Planner import *
from kRRG import *
from RRTVisualizer import *

class kRRGPlanner(Planner):
    
    def __init__(self, planning_range, segment_length, cost_func=None, map_file=None):
        super(kRRGPlanner, self).__init__(planning_range, segment_length, map_file)
        self.rrg = kRRG(self.planningRange, self.segmentLength)
        if self.mapFile!= None:
            self.rrg.loadMap(self.mapFile)
        self.krrg_viz = RRTVisualizer(self.rrg)
        self.cost_func = cost_func
        
    def findPath(self, start, goal, iterationNum):
        
        self.rrg.init(start, goal, self.cost_func)
        for i in range(iterationNum):
            self.rrg.extend()
            self.krrg_viz.update()
            
        path = self.rrg.findPath()
        
        self.krrg_viz.activePath = path
        
        self.krrg_viz.update()
        
        return path