'''
Created on Feb 1, 2015

@author: daqing_yi
'''

import shapely.geometry as shpgeo

class TrajectoryReader(object):

    def __init__(self, world_map):
        self.world_map = world_map
                    
                
        
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
            subseg = self.world_map.getSubsegment(posList[i])
            if subseg != None:
                subsegmentList.append(subseg)
                convertedPosList.append(posList[i])
        convertedPosList.append(posList[len(posList)-1])
        
        return convertedPosList, (start_region, end_region, subsegmentList)
                    
        
                
        
