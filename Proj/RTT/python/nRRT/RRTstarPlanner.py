'''
Created on Jan 4, 2015

@author: daqing_yi
'''

from Planner import *
from RRTstar import *
from RRTVisualizer import *

class RRTstarPlanner(Planner):
    
    def __init__(self, planning_range, segment_length, cost_func=None, map_file=None):
        super(RRTstarPlanner, self).__init__(planning_range, segment_length, map_file)
        self.rrts = RRTstar(self.planningRange, self.segmentLength)
        if self.mapFile!= None:
            self.rrts.loadMap(self.mapFile)
        self.rrts_viz = RRTVisualizer(self.rrts)
        self.cost_func = cost_func
        
    def findPath(self, start, goal, iterationNum):
        
        self.rrts.init(start, goal, self.cost_func)
        for i in range(iterationNum):
            self.rrts.extend()
            self.rrts_viz.update()
            
        path = self.rrts.findPath()
        
        self.rrts_viz.activePath = path
        
        self.rrts_viz.update()
        
        return path