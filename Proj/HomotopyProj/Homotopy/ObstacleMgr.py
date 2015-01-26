'''
Created on Jan 23, 2015

@author: daqing_yi
'''

import numpy as np
import shapely.geometry as shpgeo

class LineSubSegment(object):
    
    def __init__(self, line_seg, idx, parent):
        self.line_seg = line_seg
        self.idx = idx
        self.parent = parent
        
        self.name = self.parent.type + str(self.parent.parent.idx) + "-" + str(self.idx)

class LineSegmentMgr(object):
    
    def __init__(self, line_seg, type, parent):
        self.parent = parent
        self.type = type
        self.line_seg = line_seg
        self.sub_segs = []
        self.mid_points = []
        
    def load(self, obs_intsecs):
        self.obs_intsecs = obs_intsecs
        
        #print "LINE " + str(self.type) + " of OBS " + str(self.parent.idx)
        #print "INTERSECTS : " + str(self.obs_intsecs)

        # first two intersection is with self 
        if len(self.obs_intsecs) > 2:
            for i in range(2, len(self.obs_intsecs), 2):
                pos1 = self.obs_intsecs[i][2]
                pos2 = self.obs_intsecs[i+1][2]
                mid_point = ( int( (pos2[0]-pos1[0])/2 ), int( (pos2[1]-pos1[1])/2 ) )
                self.mid_points.append(mid_point)
                
        if len(self.mid_points) == 0:
            subseg = shpgeo.LineString(self.line_seg)
            lineSubSeg = LineSubSegment(subseg, 0, self)
            self.sub_segs.append(lineSubSeg)
        else:
            subseg = shpgeo.LineString([self.parent.bk, self.mid_points[0]])
            lineSubSeg = LineSubSegment(subseg, 0, self)
            self.sub_segs.append(lineSubSeg)
            
            for i in range(1, len(self.mid_points), 1):
                subseg = shpgeo.LineString([self.parent.bk, self.mid_points[len(self.mid_points)-1]])
                lineSubSeg = LineSubSegment(subseg, len(self.mid_points), self)
                self.sub_segs.append(lineSubSeg)
            
            subseg = shpgeo.LineString([self.parent.bk, self.mid_points[len(self.mid_points)-1]])
            lineSubSeg = LineSubSegment(subseg, len(self.mid_points), self)
            self.sub_segs.append(lineSubSeg)
            
        print "OBS " + str(self.parent.idx) + " " + str(self.type) + " " + str(self.sub_segs)
        
                

class ObstacleMgr(object):


    def __init__(self, contour, idx):
        self.contour = contour
        self.idx = idx
        
        self.name = "OBS"+str(self.idx)
        
        self.x_min = np.min(contour[:,0,0])
        self.x_max = np.max(contour[:,0,0])
        self.y_min = np.min(contour[:,0,1])
        self.y_max = np.max(contour[:,0,1])
        vex_list = [(self.contour[i,0,0], self.contour[i,0,1]) for i in range(self.contour.shape[0])]
        #print vex_list
        self.polygon = shpgeo.Polygon(vex_list)
        
        self.centroid = (int(self.polygon.centroid.x), int(self.polygon.centroid.y))
        
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