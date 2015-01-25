'''
Created on Jan 23, 2015

@author: daqing_yi
'''

import numpy as np
import shapely.geometry as shpgeo

class ObstacleMgr(object):


    def __init__(self, contour, idx):
        self.contour = contour
        self.idx = idx
        
        self.x_min = np.min(contour[:,0,0])
        self.x_max = np.max(contour[:,0,0])
        self.y_min = np.min(contour[:,0,1])
        self.y_max = np.max(contour[:,0,1])
        vex_list = [(self.contour[i,0,0], self.contour[i,0,1]) for i in range(self.contour.shape[0])]
        #print vex_list
        self.polygon = shpgeo.Polygon(vex_list)
        
        self.alpha_ray = None
        self.beta_ray = None
        self.alpha_seg = None
        self.beta_seg = None

        #print self.polygon
        
        self.alpha_self_intsecs = []
        self.alpha_obs_intsecs = []
        self.beta_self_intsecs = []
        self.beta_obs_intsecs = []
        
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