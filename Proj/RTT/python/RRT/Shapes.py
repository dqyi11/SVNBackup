'''
Created on 2014-11-14

@author: Walter
'''
import numpy as np
from ConnectedComponent import *

class Line(object):

    def __init__(self, point_s, point_e):
        self.point_s = point_s
        self.point_e = point_e
        self.K = float(point_s[1]-point_e[1])/float(point_s[0]-point_e[0])
        
        self.min_x = np.min([point_s[0], point_e[0]])
        self.max_x = np.max([point_s[0], point_e[0]])
        self.min_y = np.min([point_s[1], point_e[1]])
        self.max_y = np.max([point_s[1], point_e[1]])
        
        self.rad = np.arctan2(float(self.point_e[0]-self.point_s[0]), float(self.point_e[1]-self.point_s[1]))
        if self.rad < 0:
            self.rad += 2*np.pi
        
    def getY(self, x):
        return int(self.K*float(x-self.point_s[0])+self.point_s[1])
    
    def getX(self, y):
        return int(float(y-self.point_s[1])/self.K)+self.point_s[0]
    
class Region(object):
    
    def __init__(self, end1, end2, worldmap, idx):
        self.idx = idx
        self.line_s = Line(worldmap.obsCenter, end1)
        self.line_e = Line(worldmap.obsCenter, end2)
        
        self.bin_data = np.ones((worldmap.width, worldmap.height), np.int)
        self.pixels = []
        if self.line_s.rad < np.pi:
            if self.line_e.rad < np.pi:
                y_min = 0
                y_max = worldmap.obsCenter[1]
                for y in range(y_min, y_max):
                    x_min = self.line_e.getX(y)
                    x_max = self.line_s.getX(y)
                    for x in range(x_min, x_max):
                        if worldmap.isObstaclePoint(x,y)==False:
                            self.pixels.append([x,y])
                            self.bin_data[x,y] = 0
            else:
                y_min = 0
                y_max = worldmap.obsCenter[1]
                for y in range(y_min, y_max):
                    x_min = 0
                    x_max = self.line_s.getX(y)
                    for x in range(x_min, x_max):
                        if worldmap.isObstaclePoint(x,y)==False:
                            self.pixels.append([x,y])
                            self.bin_data[x,y] = 0
                y_min = worldmap.obsCenter[1]+1
                y_max = worldmap.height
                for y in range(y_min, y_max):
                    x_min = 0
                    x_max = self.line_e.getX(y)
                    for x in range(x_min, x_max):
                        if worldmap.isObstaclePoint(x,y)==False:
                            self.pixels.append([x,y])
                            self.bin_data[x,y] = 0
        else:
            if self.line_e.rad < np.pi:
                y_min = 0
                y_max = worldmap.obsCenter[1]
                for y in range(y_min, y_max):
                    x_min = self.line_e.getX(y)
                    x_max = worldmap.width
                    for x in range(x_min, x_max):
                        if worldmap.isObstaclePoint(x,y)==False:
                            self.pixels.append([x,y]) 
                            self.bin_data[x,y] = 0
                y_min = worldmap.obsCenter[1]+1
                y_max = worldmap.height
                for y in range(y_min, y_max):
                    x_min = self.line_s.getX(y)
                    x_max = worldmap.width
                    for x in range(x_min, x_max):
                        if worldmap.isObstaclePoint(x,y)==False:
                            self.pixels.append([x,y]) 
                            self.bin_data[x,y] = 0
            else:
                y_min = worldmap.obsCenter[1]+1
                y_max = worldmap.height
                for y in range(y_min, y_max):
                    x_min = self.line_s.getX(y)
                    x_max = self.line_e.getX(y)
                    for x in range(x_min, x_max):
                        if worldmap.isObstaclePoint(x,y)==False:
                            self.pixels.append([x,y])
                            self.bin_data[x,y] = 0
                            
        #generate component
        ccMgr = ConnectedComponentMgr(self.bin_data)
        self.sub_region_num = ccMgr.getComponentNum()
        self.sub_regions = []
        for idx in range(self.sub_region_num):
            self.sub_regions.append(ccMgr.getComponent(idx))


        