'''
Created on Oct 28, 2014

@author: daqing_yi
'''

neighbor_operators = [ [1, 0], [1,-1], [0,-1], [-1,-1], [-1, 0], [-1, 1], [0, 1], [1, 1] ];

class ShapeDescriptor(object):

    def __init__(self, data):
        self.data = data
        self.width = data.shape[0]
        self.height = data.shape[1]
        
    def findChainCode(self):
        
    def isBoundaryPixel(self, i, j):
    
        for no in neighbor_operators:
            nx = i + no[0]
            ny = j + no[1]
            if nx < 0 or nx >= self.width or ny < 0 or ny >= self.height:
                return True
            if self.data[nx, ny] == 0:
                return True
        return False
            
        
        
        
        
        
    def getPerimeter(self):
        return 0.0
        
    def getCompactness(self):
        return 0.0
    
    def getRectangularity(self):
        return 0.0
        
    def getEccentricity(self):
        return 0.0
    
    def getMoments(self):
        return 0.0
    
    def getElongation(self):
        return 0.0
    
    def getPsi_s_curve(self):
        return 0.0
    
    def getProfiles(self):
        return 0.0
    
    def getHoles(self):
        return 0.0
    
    def getCorners(self):
        return 0.0
    
    
        