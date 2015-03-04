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
            subseg_list = self.world_map.getCrossingSubsegments(posList[i], posList[i+1])
            if len(subseg_list) > 0:
                for subseg in subseg_list:
                    subsegmentList.append(subseg)
            convertedPosList.append(posList[i])
        convertedPosList.append(posList[len(posList)-1])
        
        return convertedPosList, (start_region, end_region, subsegmentList)
    
    def getString(self, referenceInfo):
            
        subsegs_strs = []
        for s in referenceInfo[2]:
            subsegs_strs.append(s.name)
            
        return subsegs_strs
    
    def isInCenterGroup(self, name):
        for gn in self.centerGroup:
            if gn[0] == name:
                return True
        return False
    
    def compareChar(self, nameA, nameB):
        if nameA == nameB:
            return True
        if self.isInCenterGroup(nameA)==True and self.isInCenterGroup(nameB)==True:
            return True
        return False
        
    
    def shortenString(self, strPath):
        
        newStrPath = []
        if len(strPath)==0:
            return newStrPath
        newStrPath.append(strPath[0])
        
        for i in range(1, len(strPath)):
            if len(newStrPath) > 0 and self.compareChar(newStrPath[len(newStrPath)-1], strPath[i])==True:
                newStrPath.pop()
            else:
                newStrPath.append(strPath[i])
        
        return newStrPath
                
    
    def compareStringPath(self, strPath, refStrPath, completeCompare=False):
        
        s_strPath = self.shortenString(strPath)
        s_refStrPath = self.shortenString(refStrPath)
        
        s_strPath_len = len(s_strPath)
        s_refStrPath_len = len(s_refStrPath)
        
        if s_strPath_len==0:
            if s_refStrPath_len == 0:
                return True
            else:
                return False
        
        if s_strPath_len > s_refStrPath_len:
            return False
        
        if completeCompare==True:
            if s_strPath_len != s_refStrPath_len:
                return False
        
        for i in range(len(s_strPath)):
            if self.compareChar(s_strPath[i], s_refStrPath[i]) == False:
                return False           
        return True

        
        
            
                                
                                
                    
        
                
        
