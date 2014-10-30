'''
Created on Oct 28, 2014

@author: daqing_yi
'''

from boundingShape import *
from sets import Set

neighbor_operators = [ [1, 0], [1,-1], [0,-1], [-1,-1], [-1, 0], [-1, 1], [0, 1], [1, 1] ];

class ShapeDescriptor(object):

    def __init__(self, data):
        self.data = data
        self.width = data.shape[0]
        self.height = data.shape[1]
        
    def findChainCode(self):        
        start = self.findChainCodeStart()
        cc = []
        chain = []
        dir = 0
        coord = [0, 0]
        coord[0], coord[1] = start[0], start[1]
        while True:
            newcoord = [0, 0]
            newcoord[0] = coord[0] + neighbor_operators[dir][0]
            newcoord[1] = coord[1] + neighbor_operators[dir][1]
            if True == self.isBoundaryPixel(newcoord[0], newcoord[1]):
                cc.append(dir)
                chain.append(newcoord)
                coord[0] = newcoord[0]
                coord[1] = newcoord[1]
                dir = (dir+2) % 8
                #print chain
            else:
                dir = (dir-1) % 8
            if coord[0]==start[0] and coord[1]==start[1]:
                break
            
        return cc, chain

        
    def findChainCodeStart(self):
        
        for i in range(self.width):
            for j in range(self.height):
                if self.isBoundaryPixel(i, j) == True:
                    return [i, j]
        return None
    
    def getBoundaryPixel(self):
        boundaryPixels = []
        for i in range(self.width):
            for j in range(self.height):
                if self.isBoundaryPixel(i, j) == True:
                    boundaryPixels.append( [i, j] )
        return boundaryPixels
        
        
    def isBoundaryPixel(self, i, j):
        # val = 1 means a ON pixel
        boundary = False
        if self.data[i,j] == 1:
            for no in neighbor_operators:
                nx = i + no[0]
                ny = j + no[1]
                if nx < 0 or nx >= self.width or ny < 0 or ny >= self.height:
                    return False
                if self.data[nx, ny] == 0:
                    boundary = True
        return boundary
        
    def getPerimeter(self):
        cc, chain = self.findChainCode()
        length = 0.0
        for c in cc:
            if c%2==0:
                length += 1.0
            else:
                length += 1.414
        return length
    
    def getArea(self):
        area = 0.0
        for i in range(self.width):
            for j in range(self.height):
                if self.data[i,j]==1:
                    area += 1.0
        return area
        
    def getCompactness(self):
        perimeter = self.getPerimeter()
        area = self.getArea()
        return (perimeter**2)/area
    
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
    
    def getConvexHull(self):        
        points = Set()
        for i in range(self.width):
            for j in range(self.width):
                if self.data[i,j]==1:
                    points.add((i,j))
        convexHulls = getConvexHull(points)
        return convexHulls