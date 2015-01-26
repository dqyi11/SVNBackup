'''
Created on Jan 23, 2015

@author: daqing_yi
'''

import numpy as np
import shapely.geometry as shpgeo

class LineSegmentMgr(object):
    
    def __init__(self, line_seg, type, parent):
        self.parent = parent
        self.type = type
        self.line_seg = line_seg
        self.sub_segs = []
        
    def load(self, self_intsecs, other_intsecs):
        self.self_intsecs = self_intsecs
        self.other_intsecs = other_intsecs
        
        print "LINE " + str(self.type) + " of OBS " + str(self.parent.idx)
        print "ALPHA: " + str(self.self_intsecs)
        print "BETA: " + str(self.other_intsecs)
        

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