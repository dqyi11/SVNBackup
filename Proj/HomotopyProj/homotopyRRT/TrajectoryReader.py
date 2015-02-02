'''
Created on Feb 1, 2015

@author: daqing_yi
'''

import shapely.geometry as shpgeo

class TrajectoryReader(object):

    def __init__(self, world_map):
        self.world_map = world_map
        self.centerGroup = []    
        
        self.initEquSubsegs()
    
    def initEquSubsegs(self):
        
        for s in self.world_map.subsegments:
            if s.isConnectedToCentralPoint == True:
                self.centerGroup.append((s.name, s))
            
        
    def readPath(self, posList):
        if len(posList)==0:
            return None
        
        start = shpgeo.Point(posList[0][0], posList[0][1])
        end = shpgeo.Point(posList[len(posList)-1][0], posList[len(posList)-1][1])
        
        start_region = None
        end_region = None
        
        for reg in self.world_map.regions:
            for subreg in reg.subregions:
                if subreg.polygon.contains(start):
                    start_region = subreg
                if subreg.polygon.contains(end):
                    end_region = subreg
        
        convertedPosList = []
        subsegmentList = []
        convertedPosList.append(posList[0])  
        for i in range(1, len(posList)-1):
            subseg = self.world_map.getCrossingSubsegment(posList[i], posList[i+1])
            if subseg != None:
                subsegmentList.append(subseg)
                convertedPosList.append(posList[i])
        convertedPosList.append(posList[len(posList)-1])
        
        return convertedPosList, (start_region, end_region, subsegmentList)
    
    def getString(self, referenceInfo):
        
        start_str = referenceInfo[0].getName()
        end_str = referenceInfo[1].getName()
        
        subsegs_strs = []
        for s in referenceInfo[2]:
            subsegs_strs.append(s.name)
            
        return (start_str, end_str, subsegs_strs)
    
    def isInCenterGroup(self, name):
        for gn in self.centerGroup:
            if gn[0] == name:
                return True
        return False
    
    def compareStr(self, nameA, nameB):
        if nameA == nameB:
            return True
        if self.isInCenterGroup(nameA)==True and self.isInCenterGroup(nameB)==True:
            return True
        return False
        
    
    def shortenString(self, strPath):
        
        newStrPath = []
        newStrPath.append(strPath[0])
        
        for i in range(1, len(strPath)):
            if self.compareStr(newStrPath[len(newStrPath)-1], strPath[i])==False:
                newStrPath.append(strPath[i])
                
        return newStrPath
                
    
    def compareStringPath(self, strPath, refStrPath):
        
        s_strPath = self.shortenString(strPath)
        s_refStrPath = self.shortenString(refStrPath)
        
        if len(s_strPath) > len(s_refStrPath):
            return False
        
        for i in range(len(s_strPath)):
            if self.compareStr(s_strPath[i], s_refStrPath[i]) == False:
                return False           
        return True
            
            
            
                                
                                
                    
        
                
        
