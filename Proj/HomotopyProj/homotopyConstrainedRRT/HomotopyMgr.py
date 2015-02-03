'''
Created on Feb 2, 2015

@author: daqing_yi
'''
from TrajectoryReader import *
from WorldMapMgr import *
import copy

class HomotopyMgr(object):

    def __init__(self, world_map, reader):
        self.world_map = world_map
        self.reader = reader
        self.refStrPath = None
        
    def init(self, refStrPath):
        self.refStrPath = self.reader.shortenString(refStrPath)
        
    def inSameHomotopy(self, path):
        return self.reader.compareStringPath(path, self.refStrPath)
    
    def extendPath(self, currentPath, start, end):
        
        newPath = copy.deepcopy(currentPath)
        subseg = self.world_map.getCrossingSubsegment(start, end)
        if subseg != None:
            if len(newPath) > 0:
                if subseg.name != newPath[len(newPath)-1]:
                    newPath.append(subseg.name)
            else:
                newPath.append(subseg.name)
        return newPath
        