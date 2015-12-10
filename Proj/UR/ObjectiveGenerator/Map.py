'''
Created on Dec 9, 2015

@author: walter
'''

from scipy.misc import imread
import numpy as np

THRESHOLD = 10

class Map(object):

    def __init__(self, filename):
        self.pixmap = np.array( imread( filename, True ))
        self.width = self.pixmap.shape[1]
        self.height = self.pixmap.shape[0]
        self.triangle_length = int(np.sqrt( self.width**2 + self.height**2 ))
        
    def is_in_obstacle(self, x, y):
        if self.pixmap[y,x] > THRESHOLD:
            return False
        return True 

    def is_in_range(self, x, y):
        if x >= 0 and x < self.width and y >= 0 and y < self.height:
            return True
        return False
    
    def get_circle(self, x, y, r):
        points = []
        p_x = r
        p_y = 0
        decisionOver2 = 1 - p_x
        while p_y <= p_x:
            points.append( ( p_x + x, p_y + y ) )
            points.append( ( p_y + x, p_x + y ) )
            points.append( ( -p_x + x, p_y + y ) )
            points.append( ( -p_y + x, p_x + y ) )
            points.append( ( -p_x + x, -p_y + y ) )
            points.append( ( -p_y + x, -p_x + y ) )
            points.append( ( p_x + x, -p_y + y ) )
            points.append( ( p_y + x, -p_x + y ) )
            p_y += 1
            if decisionOver2 <= 0:
                decisionOver2 += 2 * p_y + 1
            else:
                p_x -= 1
                decisionOver2 += 2 * (p_y - p_x) + 1
        new_points = []
        for p in points:
            if self.is_in_range( p[0], p[1] ):
                new_points.append( ( p[0], p[1] ) ) 
        return new_points
                
    def intersect_with_obstacle(self, points):
        for p in points:
            if True == self.is_in_obstacle(p[0], p[1]):
                return True
        return False
    
    def dist_to_nearest_obstacle(self, x, y):
        for r in range(1, self.triangle_length):
            points = self.get_circle(x, y, r)
            #print str(x) + "-" + str(y) + "-" + str(r)
            if True == self.intersect_with_obstacle(points):
                return r
        return self.triangle_length