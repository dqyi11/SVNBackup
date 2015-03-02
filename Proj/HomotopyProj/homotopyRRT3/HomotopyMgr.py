'''
Created on Feb 2, 2015

@author: daqing_yi
'''
from TrajectoryReader import *
from WorldMapMgr import *
import copy
import numpy as np

class HomotopyMgr(object):

    def __init__(self, world_map, reader):
        self.world_map = world_map
        self.reader = reader
        self.refStrPath = None
        
        self.dividingRefs = []
        self.startReg = None
        self.endReg = None
        
        self.allHomotopyClasses = []
        
    def init(self, refStrPath, startReg, endReg):
        self.refStrPath = refStrPath
        self.refStrPathShort = self.reader.shortenString(refStrPath)
        self.startReg = startReg
        self.endReg = endReg
        self.allHomotopyClasses = self.world_map.getTopologicalGraph().findAllPathsByBFS(self.startReg.getName(), self.endReg.getName())
        print "REF"
        print self.refStrPath
        print "ALL " + str(len(self.allHomotopyClasses))
        print self.refStrPath in self.allHomotopyClasses
        print self.allHomotopyClasses
        
    def inSameHomotopy(self, path):
        return self.reader.compareStringPath(path, self.refStrPath)
    
    def extendPath(self, currentPath, start, end):
        
        newPath = copy.deepcopy(currentPath)
        newBit = []
        subseg_list = self.world_map.getCrossingSubsegments(start, end)
        if len(subseg_list) > 0:
            for subseg in subseg_list:
                if len(newPath) > 0:
                    if subseg.name != newPath[len(newPath)-1]:
                        newPath.append(subseg.name)
                        newBit.append( subseg.name )
                else:
                    newPath.append(subseg.name)
                    newBit.append( subseg.name )
        return newPath, newBit
    
    def isCrossingDividingRefs(self, start, end):
        intseg = self.world_map.getCrossingSubsegList(start, end, self.dividingRefs)
        if intseg == None:
            return False
        return True
    
    def getDividingRefs(self, start, end):
        rad_start = np.arctan2(start[1]-self.world_map.centralPoint[1], start[0]-self.world_map.centralPoint[0])
        if rad_start < 0:
            rad_start += 2*np.pi
        rad_end = np.arctan2(end[1]-self.world_map.centralPoint[1], end[0]-self.world_map.centralPoint[0])
        if rad_end < 0:
            rad_end += 2*np.pi
            
        if rad_start < rad_end:
            rad1 = rad_start
            rad2 = rad_end
        else:
            rad1 = rad_end
            rad2 = rad_start
        
        between_rads = []
        for rr in self.world_map.rad_list:
            if rr >= rad1 and rr < rad2:
                between_rads.append(rr)
                
        between_rads.sort(reverse=False)
        
        ref_rad = None
        if len(between_rads) > 0:
            ref_rad = between_rads[int(len(between_rads)/2)]

        print "GET DIVIDING REFS "
        print ref_rad
        
        precisionDelta = 0.07
        self.dividingRefs = []
        for subseg in self.world_map.subsegments:
            if (subseg.rad>=ref_rad-precisionDelta and subseg.rad<ref_rad+precisionDelta) \
            or (subseg.rad>=ref_rad+np.pi-precisionDelta and subseg.rad<ref_rad+np.pi+precisionDelta) \
            or (subseg.rad>=ref_rad-np.pi-precisionDelta and subseg.rad<ref_rad-np.pi+precisionDelta):
                self.dividingRefs.append(subseg)
                
        return self.dividingRefs
                
        
        
        