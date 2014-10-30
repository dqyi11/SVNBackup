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
        """Computes the convex hull of a set of 2D points.
     
        Input: an iterable sequence of (x, y) pairs representing the points.
        Output: a list of vertices of the convex hull in counter-clockwise order,
          starting from the vertex with the lexicographically smallest coordinates.
        Implements Andrew's monotone chain algorithm. O(n log n) complexity.
        """
     
        # Sort the points lexicographically (tuples are compared lexicographically).
        # Remove duplicates to detect the case we have just one unique point.
        points = sorted(set(points))
     
        # Boring case: no points or a single point, possibly repeated multiple times.
        if len(points) <= 1:
            return points
     
        # 2D cross product of OA and OB vectors, i.e. z-component of their 3D cross product.
        # Returns a positive value, if OAB makes a counter-clockwise turn,
        # negative for clockwise turn, and zero if the points are collinear.
        def cross(o, a, b):
            return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
     
        # Build lower hull 
        lower = []
        for p in points:
            while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
                lower.pop()
            lower.append(p)
     
        # Build upper hull
        upper = []
        for p in reversed(points):
            while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
                upper.pop()
            upper.append(p)
     
        # Concatenation of the lower and upper hulls gives the convex hull.
        # Last point of each list is omitted because it is repeated at the beginning of the other list. 
        return lower[:-1] + upper[:-1]
        
    
    
        