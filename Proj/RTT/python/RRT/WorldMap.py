'''
Created on Nov 3, 2014

@author: daqing_yi
'''

from PIL import Image
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from ConnectedComponent import *
import csv

class Obstacle(object):
    
    def __init__(self, idx):
        self.idx = idx
        self.pixels = []
        
    def samplePosition(self):
        rndIdx = np.random.randint(len(self.pixels))
        return self.pixels[rndIdx]
    
    def getCenter(self):
        mX, mY = 0.0, 0.0
        for p in self.pixels:
            mX += p[0]
            mY += p[1]
        mX /= len(self.pixels)
        mY /= len(self.pixels)
        return [mX, mY]
    
class ReferenceFrameManager(object):
    
    def __init__(self, center_pos, obs_pos):
        self.center_pos = center_pos
        self.obs_pos = obs_pos
        self.k = float(self.obs_pos[0]-self.center_pos[0])/(self.obs_pos[1]-self.center_pos[1])
        print self.k
        self.quarant = 0
        if self.center_pos[0] < self.obs_pos[0]:
            if self.center_pos[1] >= self.obs_pos[1]:
                self.quarant = 4
        else:
            if self.center_pos[1] >= self.obs_pos[1]:
                self.quarant = 3
            else:
                self.quarant = 2
        

class WorldMap(object):

    def __init__(self):
        self.obstacle_num = 0
        self.obstacles = []

    def load(self, map_file):
        img = Image.open(map_file).convert("L")
        self.width = img.size[0]
        self.height = img.size[1]
        self.bin_data = np.zeros((self.width, self.height), np.int)
        for i in range(self.width):
            for j in range(self.height):
                if img.getpixel((i,j)) <= 122:
                    self.bin_data[i,j] = 0
                else:
                    self.bin_data[i,j] = 1
        
        ccMgr = ConnectedComponentMgr(self.bin_data)
        self.obstacle_num = ccMgr.getComponentNum()
        for idx in range(self.obstacle_num):
            obs = Obstacle(idx)
            obs.pixels = ccMgr.getComponent(idx)
            self.obstacles.append(obs)
            
        self.obsKeyPoints = []
        for obs in self.obstacles:
            self.obsKeyPoints.append(obs.samplePosition())
            
        
        mX, mY = 0.0, 0.0
        for obs in self.obstacles:
            c = obs.getCenter()
            mX += c[0]
            mY += c[1]
        mX /= len(self.obstacles)
        mY /= len(self.obstacles)
        self.obsCenter = [int(mX), int(mY)]
        
        self.rf_mgrs = []
        for op in self.obsKeyPoints:
            rf_mgr = ReferenceFrameManager(op, self.obsCenter)
            self.rf_mgrs.append(rf_mgr)
        
        
    def dump(self, filename):
        with open(filename, 'wb') as f:
            writer = csv.writer(f)
            writer.writerows(self.bin_data)
        
    def visualize(self):        
        obs_color = (0.5,0.5,0.5)
        color_img = np.ones((self.height, self.width, 3), np.float)
        
        for obs in self.obstacles:
            for o in obs.pixels:
                #print o
                color_img[o[1], o[0], 0] = obs_color[0]
                color_img[o[1], o[0], 1] = obs_color[1]
                color_img[o[1], o[0], 2] = obs_color[2]
            
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.imshow(color_img)
        
        for op in self.obsKeyPoints:
            ax.plot(op[0],op[1],'ro')
        ax.plot(self.obsCenter[0], self.obsCenter[1], 'bo')
        
        plt.show()
        