'''
Created on Jan 4, 2015

@author: daqing_yi
'''

from RRTstar import *
from RRTVisualizer import *

class RRTstarPlanner(object):
    
    def __init__(self, planning_range, segment_length, cost_func=None, map_file=None):
        self.planningRange = planning_range
        self.segmentLength = segment_length
        self.mapFile = map_file
        self.rrts = RRTstar(self.planningRange, self.segmentLength)
        if self.mapFile!= None:
            self.rrts.loadMap(self.mapFile)
        self.rrts_viz = RRTVisualizer(self.rrts)
        self.cost_func = cost_func
        
    def findPath(self, start, goal, iterationNum):
        
        self.rrts.init(start, goal, self.cost_func)
        for i in range(iterationNum):
            print "Iter@" + str(i)
            self.rrts.extend()
            self.rrts_viz.update()
            
        path = self.rrts.findPath()
        
        self.rrts_viz.activePath = path
        
        self.rrts_viz.update()
        
        return path