'''
Created on Dec 30, 2014

@author: daqing_yi
'''

from Planner import *
from RRT import *
from RRTVisualizer import *

class RRTPlanner(Planner):
    
    def __init__(self, planning_range, segment_length, map_file=None):
        super(RRTPlanner, self).__init__(planning_range, segment_length, map_file)
        self.rrt = RRT(self.planningRange, self.segmentLength)
        if self.mapFile!= None:
            self.rrt.loadMap(self.mapFile)
        self.rrt_viz = RRTVisualizer(self.rrt)
        
    def findPath(self, start, goal, iterationNum):
        
        self.rrt.init(start, goal)
        for i in range(iterationNum):
            self.rrt.extend()
            self.rrt_viz.update()
            
        path = self.rrt.findPath()
        
        self.rrt_viz.activePath = path
        
        self.rrt_viz.update()
        
        return path