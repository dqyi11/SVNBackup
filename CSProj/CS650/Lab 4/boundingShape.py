'''
Created on 2014-10-30

@author: Walter
'''

import numpy as np

def cross(o, a, b):
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

def getConvexHull(points): 
    points = sorted(set(points))
    lower = []
    for p in points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(points):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]
 
    
def getMinimumBoundingBox(points):
    
    hull_points_2d = np.array(getConvexHull(points))

    edges = np.zeros( (len(hull_points_2d)-1,2) ) 
    for i in range( len(edges) ):
        edge_x = hull_points_2d[i+1, 0] - hull_points_2d[i, 0]
        edge_y = hull_points_2d[i+1, 1] - hull_points_2d[i, 1]
        edges[i] = [edge_x,edge_y]

    edge_angles = np.zeros( (len(edges)) )
    for i in range( len(edge_angles) ):
        edge_angles[i] = np.arctan2( edges[i,1], edges[i,0] )
    for i in range( len(edge_angles) ):
        edge_angles[i] = abs( edge_angles[i] % (np.pi/2) ) 
    edge_angles = np.unique(edge_angles)
 
    min_area = np.inf
    angle = 0
    width = 0
    height = 0
    min_x, min_y, max_x, max_y = 0, 0, 0, 0
    for i in range( len(edge_angles) ):
        R = np.array([ [ np.cos(edge_angles[i]), np.cos(edge_angles[i]-(np.pi/2)) ], [ np.cos(edge_angles[i]+(np.pi/2)), np.cos(edge_angles[i]) ] ])
        rot_points = np.dot(R, np.transpose(hull_points_2d) ) 
        t_min_x = np.nanmin(rot_points[0], axis=0)
        t_max_x = np.nanmax(rot_points[0], axis=0)
        t_min_y = np.nanmin(rot_points[1], axis=0)
        t_max_y = np.nanmax(rot_points[1], axis=0)

        width = t_max_x - t_min_x
        height = t_max_y - t_min_y
        area = width*height

        if area < min_area:
            angle = edge_angles[i]
            min_area = area
            min_x = t_min_x
            min_y = t_min_y
            max_x = t_max_x
            max_y = t_max_y

    R = np.array([ [ np.cos(angle), np.cos(angle-(np.pi/2)) ], [ np.cos(angle+(np.pi/2)), np.cos(angle) ] ])
    corner_points = np.zeros( (4,2) )
    corner_points[0] = np.dot( [ max_x, min_y ], R )
    corner_points[1] = np.dot( [ min_x, min_y ], R )
    corner_points[2] = np.dot( [ min_x, max_y ], R )
    corner_points[3] = np.dot( [ max_x, max_y ], R )

    return corner_points
    
    
        
