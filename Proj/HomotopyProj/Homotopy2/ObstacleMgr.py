'''
Created on Jan 23, 2015

@author: daqing_yi
'''

import numpy as np
import shapely.geometry as shpgeo

class LineSubSegment(object):
    
    def __init__(self, line_seg, open_seg, idx, parent):
        self.line_seg = line_seg
        self.open_seg = open_seg
        self.idx = idx
        self.parent = parent
        
        self.name = self.parent.type + str(self.parent.parent.idx) + "-" + str(self.idx)
        self.midpoint = ( (open_seg[1][0]+open_seg[0][0])/2, (open_seg[1][1]+open_seg[0][1])/2 )
        
        self.regionAInfo = None
        self.regionBInfo = None
        
        self.checkPosA = None
        self.checkPosB = None
    
        
        self.checkRegionA = None
        self.checkRegionB = None
        
        self.isConnectedToCentralPoint = False

class LineSegmentMgr(object):
    
    def __init__(self, line_seg, type, parent):
        self.parent = parent
        self.type = type
        self.line_seg = line_seg
        self.sub_segs = []
        self.mid_points = []
        
        self.from_pos = line_seg.coords[0]
        self.end_pos = line_seg.coords[1]
        
    def load(self, obs_intsecs):
        self.obs_intsecs = obs_intsecs
        
        #print "LINE " + str(self.type) + " of OBS " + str(self.parent.idx)
        #print "INTERSECTS : " + str(self.obs_intsecs)
        if self.type=="A":
            cpdist = np.sqrt((self.parent.bk[0]-self.parent.parent.centralPoint[0])**2+(self.parent.bk[1]-self.parent.parent.centralPoint[1])**2)
            
            idx = 0
            for i in range(1, len(self.obs_intsecs)-1, 2):
                sec1 = self.obs_intsecs[i]
                sec2 = self.obs_intsecs[i+1]
                if sec1[3] <= cpdist and sec2[3] >= cpdist:
                    open_seg = (sec1[2], self.parent.parent.centralPoint)
                    subseg = shpgeo.LineString([self.obs_intsecs[i-1][2], self.parent.parent.centralPoint])
                    lineSubSeg = LineSubSegment(subseg, open_seg, idx, self)
                    self.sub_segs.append(lineSubSeg)
                    idx += 1
                    
                    open_seg = (self.parent.parent.centralPoint, sec2[2])
                    subseg = shpgeo.LineString([self.parent.parent.centralPoint, sec2[2]])
                    lineSubSeg = LineSubSegment(subseg, open_seg, idx, self)
                    self.sub_segs.append(lineSubSeg)
                    idx += 1 
                else:
                    open_seg = (sec1[2], sec2[2])
                    subseg = shpgeo.LineString([self.obs_intsecs[i-1][2], sec2[2]])
                    lineSubSeg = LineSubSegment(subseg, open_seg, idx, self)
                    self.sub_segs.append(lineSubSeg)
                    idx += 1
            
            if self.obs_intsecs[len(self.obs_intsecs)-1][3] > cpdist:        
                open_seg = (self.obs_intsecs[len(self.obs_intsecs)-1][2], self.end_pos)
                subseg = shpgeo.LineString([self.obs_intsecs[len(self.obs_intsecs)-2][2], self.end_pos])
                lineSubSeg = LineSubSegment(subseg, open_seg, idx, self)
                self.sub_segs.append(lineSubSeg)
                idx += 1
            else:
                open_seg = (self.obs_intsecs[len(self.obs_intsecs)-1][2], self.parent.parent.centralPoint)
                subseg = shpgeo.LineString([self.obs_intsecs[len(self.obs_intsecs)-2][2], self.parent.parent.centralPoint])
                lineSubSeg = LineSubSegment(subseg, open_seg, idx, self)
                self.sub_segs.append(lineSubSeg)
                idx += 1
                
                open_seg = (self.parent.parent.centralPoint, self.end_pos)
                subseg = shpgeo.LineString([self.parent.parent.centralPoint, self.end_pos])
                lineSubSeg = LineSubSegment(subseg, open_seg, idx, self)
                self.sub_segs.append(lineSubSeg)
                idx += 1 
            
        elif self.type=="B":
            
            idx = 0
            for i in range(1, len(self.obs_intsecs)-1, 2):
                sec1 = self.obs_intsecs[i]
                sec2 = self.obs_intsecs[i+1]

                open_seg = (sec1[2], sec2[2])
                subseg = shpgeo.LineString([self.obs_intsecs[i-1][2], sec2[2]])
                lineSubSeg = LineSubSegment(subseg, open_seg, idx, self)
                self.sub_segs.append(lineSubSeg)
                idx += 1
                
            open_seg = (self.obs_intsecs[len(self.obs_intsecs)-1][2], self.end_pos)
            subseg = shpgeo.LineString([self.obs_intsecs[len(self.obs_intsecs)-2][2], self.end_pos])
            lineSubSeg = LineSubSegment(subseg, open_seg, idx, self)
            self.sub_segs.append(lineSubSeg)
            idx += 1
        

class ObstacleMgr(object):

    def __init__(self, contour, idx, parent):
        self.contour = contour
        self.idx = idx
        self.parent = parent
        
        self.name = "OBS"+str(self.idx)
        
        self.x_min = np.min(contour[:,0,0])
        self.x_max = np.max(contour[:,0,0])
        self.y_min = np.min(contour[:,0,1])
        self.y_max = np.max(contour[:,0,1])
        vex_list = [(self.contour[i,0,0], self.contour[i,0,1]) for i in range(self.contour.shape[0])]
        #print vex_list
        self.polygon = shpgeo.Polygon(vex_list)
        
        if self.polygon.is_valid == False:
            #print "fixing polygon of OBS " + str(self.idx)
            fixed_polygon = self.polygon.buffer(0)
            #print "After fixing: " + str(fixed_polygon.is_valid)
            if fixed_polygon.type == "MultiPolygon":
                polygon_list = []
                for poly in fixed_polygon:
                    polygon_list.append((poly, poly.area))
                polygon_list.sort(key=lambda x: x[1], reverse=True)
                self.polygon = polygon_list[0][0]
            elif fixed_polygon.type == "Polygon":
                self.polygon = fixed_polygon
        
        self.bk = None
        
        self.centroid = (self.polygon.centroid.x, self.polygon.centroid.y)
        
        self.alpha_ray = None
        self.beta_ray = None
        self.alpha_seg = None
        self.beta_seg = None

        #print self.polygon
        
        self.alpha_self_intsecs = []
        self.alpha_obs_intsecs = []
        self.beta_self_intsecs = []
        self.beta_obs_intsecs = []
        
        self.alpha_self_intsecs_info = []
        self.beta_self_intsecs_info = []
        self.alpha_obs_intsecs_info = []
        self.beta_obs_intsecs_info = []
        
    def samplePosition(self):
        
        rndPos = None
        while rndPos==None:
            x_rnd = np.random.randint(self.x_min, self.x_max)
            y_rnd = np.random.randint(self.y_min, self.y_max)
            
            if self.polygon.contains(shpgeo.Point(x_rnd, y_rnd)):
                rndPos = (x_rnd, y_rnd)
            
        return rndPos