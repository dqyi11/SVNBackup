'''
Created on 2014-11-14

@author: Walter
'''
import numpy as np

class Line(object):

    def __init__(self, point_a, point_b):
        self.point_a = point_a
        self.point_b = point_b
        self.K = float(point_a[1]-point_b[1])/float(point_a[0]-point_b[0])
        
        self.min_x = np.min([point_a[0], point_b[0]])
        self.max_x = np.max([point_a[0], point_b[0]])
        self.min_y = np.min([point_a[1], point_b[1]])
        self.max_y = np.max([point_a[1], point_b[1]])
        
    def getY(self, x):
        return int(self.K*float(x-self.point_a[0])+self.point_a[1])
    
    def getX(self, y):
        return int(float(y-self.point_a[1])/self.K)+self.point_a[0]

        