'''
Created on Jan 25, 2015

@author: daqing_yi
'''

import shapely.geometry as shpgeo
import shapely.ops as shpops
import numpy as np

class SubRegion(object):
    
    def __init__(self, polygon):
        self.polygon = polygon
        self.neighbor_info = []

class RegionMgr(object):


    def __init__(self, ray1Info, ray2Info, idx, parent):
        self.idx = idx
        self.ray1Info = ray1Info
        self.ray2Info = ray2Info
        self.parent = parent
        self.name = "REG"+str(self.idx)
        
        self.ray1Obs = parent.obstacles[ray1Info[0]]
        self.ray2Obs = parent.obstacles[ray2Info[0]]
        
        if self.ray1Info[1] == "A":
            self.line1 = self.ray1Obs.alpha_seg
            self.line1_info = self.ray1Obs.alpha_seg_info
            #self.line1_obs_intsecs_info = self.ray1Obs.alpha_obs_intsecs_info
        else:
            self.line1 = self.ray1Obs.beta_seg
            self.line1_info = self.ray1Obs.beta_seg_info
            #self.line1_obs_intsecs_info = self.ray1Obs.beta_obs_intsecs_info
        if ray2Info[1] == "A":
            self.line2 = self.ray2Obs.alpha_seg
            self.line2_info = self.ray2Obs.alpha_seg_info
            #self.line2_obs_intsecs_info = self.ray2Obs.alpha_obs_intsecs_info
        else:
            self.line2 = self.ray2Obs.beta_seg
            self.line2_info = self.ray2Obs.beta_seg_info
            #self.line2_obs_intsecs_info = self.ray2Obs.beta_obs_intsecs
            
        pointString = self.getPointString(parent.centralPoint, self.line1_info, self.line2_info)
        self.polygon = shpgeo.Polygon(pointString)
        
        self.centroid = (int(self.polygon.centroid.x), int(self.polygon.centroid.y))
        
        current_polygon = self.polygon
        for obs in self.parent.obstacles:
            if current_polygon.intersects(obs.polygon):
                #print str(self.polygon) + " INTERSETS WITH " + str(obs.idx)
                current_polygon = current_polygon.difference(obs.polygon)
                print "Difference: " + current_polygon.type + " " + str(current_polygon)
                
        self.subregions = []
        if current_polygon.type == "Polygon":
            subregion = SubRegion(current_polygon)
            self.subregions.append(subregion)           
        elif current_polygon.type == "MultiPolygon":
            for poly in current_polygon:
                subregion = SubRegion(poly)
                self.subregions.append(subregion)
                
        # init alpha and beta segments for subregion
        
        #print self.line1_obs_intsecs_info
        #print self.line2_obs_intsecs_info   
        print "CHECK NEIGHBOR OF REGION " + str(self.name)
        for subregion in self.subregions:
            for subseg in self.line1.sub_segs:
                if subregion.polygon.intersects(subseg.line_seg):
                    print "INTERSECTS " + str(subregion) + " AND " + str(subseg) 
                    subregion.neighbor_info.append((self.line1, subseg))
        
            for subseg in self.line2.sub_segs:
                if subregion.polygon.intersects(subseg.line_seg):
                    print "INTERSECTS " + str(subregion) + " AND " + str(subseg) 
                    subregion.neighbor_info.append((self.line2, subseg))
                
        
        
        
    def getPointString(self, start, line1_info, line2_info):
        
        pointString = []
        pointString.append(start)
        pointString.append(line1_info[0])
        if line2_info[1] > line1_info[1]:
            for ccl_info in self.parent.center_corner_lines_info:
                if ccl_info[1] > line1_info[1] and ccl_info[1] < line2_info[1]:
                    pointString.append(ccl_info[0])
        else:
            for ccl_info in self.parent.center_corner_lines_info:
                if ccl_info[1] > line1_info[1]:
                    pointString.append(ccl_info[0])
            for ccl_info in self.parent.center_corner_lines_info:
                if ccl_info[1] >= 0 and ccl_info[1] < line2_info[1]:
                    pointString.append(ccl_info[0])

        pointString.append(line2_info[0])
        
        return pointString
                   
        
            
            
            
        
            

        