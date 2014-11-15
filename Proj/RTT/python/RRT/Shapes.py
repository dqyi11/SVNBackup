'''
Created on 2014-11-14

@author: Walter
'''

class Line(object):

    def __init__(self, point_a, point_b):
        self.point_a = point_a
        self.point_b = point_b
        self.K = float(point_a[1]-point_b[1])/float(point_a[0]-point_b[0])
        
    def getY(self, x):
        return int(self.K*float(x-self.point_a[0])+self.point_a[1])
    
    def getX(self, y):
        return int(float(y-self.point_a[1])/self.K)+self.point_a[0]

        