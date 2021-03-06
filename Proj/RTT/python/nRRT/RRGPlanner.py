'''
Created on Jan 4, 2015

@author: daqing_yi
'''

from Planner import *
from RRG import *
from RRTVisualizer import *

class RRGPlanner(Planner):
    
    def __init__(self, planning_range, segment_length, cost_func=None, map_file=None):
        super(RRGPlanner, self).__init__(planning_range, segment_length, map_file)
        self.rrg = RRG(self.planningRange, self.segmentLength)
        if self.mapFile!= None:
            self.rrg.loadMap(self.mapFile)
        self.rrg_viz = RRTVisualizer(self.rrg)
        self.cost_func = cost_func
        
    def findPath(self, start, goal, iterationNum):
        
        self.rrg.init(start, goal, self.cost_func)
        for i in range(iterationNum):
            self.rrg.extend()
            self.rrg_viz.update()
            
        path = self.rrg.findPath()
        
        self.rrg_viz.activePath = path
        
        self.rrg_viz.update()
        
        return path