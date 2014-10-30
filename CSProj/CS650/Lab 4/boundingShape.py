'''
Created on 2014-10-30

@author: Walter
'''

import numpy as np

def getConvexHull(points):
 
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
    
def findFurthest(j, n, s, c, mx, my, hull): # advance j to extreme point
    xn, yn = hull[j][0], hull[j][1]
    rx, ry = xn*c - yn*s, xn*s + yn*c
    best = mx*rx + my*ry
    while True:
        x, y = rx, ry
        xn, yn = hull[(j+1)%n][0], hull[(j+1)%n][1]
        rx, ry = xn*c - yn*s, xn*s + yn*c
        if mx*rx + my*ry >= best:
            j = (j+1)%n
            best = mx*rx + my*ry
        else:
            return (x, y, j)

 
    
def getMinimumBoundingBox(points):
    
    hull = getConvexHull(points)
    n = len(hull)
    iL = iR = iP = 1                # indexes left, right, opposite
    # add after pi = ... line:
    minRect = (1e33, 0, 0, 0, 0, 0, 0) # area, dx, dy, i, iL, iP, iR
    for i in range(n-1):
        dx = hull[i+1][0] - hull[i][0]
        dy = hull[i+1][1] - hull[i][1]
        theta = np.pi-np.arctan2(dy, dx)
        s, c = np.sin(theta), np.cos(theta)
        yC = hull[i][0]*s + hull[i][1]*c
    
        xP, yP, iP = findFurthest(iP, n, s, c, 0, 1, hull)
        if i==0:
            iR = iP
        xR, yR, iR = findFurthest(iR, n, s, c,  1, 0, hull)
        xL, yL, iL = findFurthest(iL, n, s, c, -1, 0, hull)
        area = (yP-yC)*(xR-xL)
        print '    {:2d} {:2d} {:2d} {:2d} {:9.3f}'.format(i, iL, iP, iR, area)   
        # add after area = ... line:
        if area < minRect[0]:
            minRect = (area, xR-xL, yP-yC, i, iL, iP, iR)
            
    # add after print ... line:
    print 'Min rectangle:', minRect
    # or instead of that print, add:
    print 'Min rectangle: ',
    for x in ['{:3d} '.format(x) if isinstance(x, int) else '{:7.3f} '.format(x) for x in minRect]:
        print x,
    print
    
    
    #return None
    
    
        
