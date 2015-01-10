'''
Created on Jan 4, 2015

@author: daqing_yi
'''

from kRRTstar import *
from RRTVisualizer import *

class kRRTstarPlanner(object):
    
    def __init__(self, planning_range, segment_length, cost_func=None, map_file=None):
        self.planningRange = planning_range
        self.segmentLength = segment_length
        self.mapFile = map_file
        self.krrts = kRRTstar(self.planningRange, self.segmentLength)
        if self.mapFile!= None:
            self.rrts.loadMap(self.mapFile)
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