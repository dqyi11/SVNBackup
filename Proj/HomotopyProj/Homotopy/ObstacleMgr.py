'''
Created on Jan 23, 2015

@author: daqing_yi
'''

import numpy as np
from shapely.geometry import Polygon, Point

class ObstacleMgr(object):


    def __init__(self, contour):
        self.contour = contour
        
        self.x_min = np.min(contour[:,0,0])
        self.x_max = np.max(contour[:,0,0])
        self.y_min = np.min(contour[:,0,1])
        self.y_max = np.max(contour[:,0,1])
        self.polygon = Polygon([(self.contour[i,0,0], self.contour[i,0,1]) for i in range(self.contour.shape[0])])

        print self.polygon
        
    def samplePosition(self):
        
        rndPos = None
        while rndPos==None:
            x_rnd = np.random.randint(self.x_min, self.x_max)
            y_rnd = np.random.randint(self.y_min, self.y_max)
            
            if self.polygon.contains(Point(x_rnd, y_rnd)):
                rndPos = (x_rnd, y_rnd)
            
        return rndPos