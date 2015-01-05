'''
Created on Dec 30, 2014

@author: daqing_yi
'''

class Planner(object):


    def __init__(self, planning_range, segment_length, map_file=None):
        self.planningRange = planning_range
        self.segmentLength = segment_length
        self.mapFile = map_file
    
    
    def findPath(self, start, goal, iterationNum):
        pass