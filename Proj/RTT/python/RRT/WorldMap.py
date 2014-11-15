'''
Created on Nov 3, 2014

@author: daqing_yi
'''

from PIL import Image
import numpy as np
from ConnectedComponent import *
import csv
import pygame, sys
from pygame.locals import *

class Obstacle(object):
    
    def __init__(self, idx):
        self.idx = idx
        self.pixels = []
        self.center = None
        self.keypoint = None
        self.alpha_line = None
        self.beta_line = None
        
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
        return [int(mX), int(mY)]
    
    def initKeypoint(self):
        self.center = self.getCenter()
        self.keypoint = self.samplePosition()
        
    def initLine(self, center):
        K = float(self.center[1]-self.keypoint[1])/(self.center[0]-self.keypoint[0])
        y_minx = int(center[1] - K * center[0])
        x_miny = int(center[0] - float(center[1])/K)
        y_maxx = int(center[1] + K * (self.width - center[0]))
        x_maxy = int(center[0] + float(self.height-center[1])/K)
        if center[0] <= self.keypoint[0]:
            if center[1] <= self.keypoint[1]:
                #mode = 4   keypoint in 4th   
                if y_maxx < self.height:
                    self.alpha_line = [[self.keypoint[0],self.keypoint[1]], [0, y_maxx]]
                else:
                    self.alpha_line = [[self.keypoint[0],self.keypoint[1]], [x_maxy, 0]]
                
                if y_minx > 0:
                    self.beta_line = [[self.keypoint[0],self.keypoint[1]], [0, y_minx]]
                else:
                    self.beta_line = [[self.keypoint[0],self.keypoint[1]], [x_miny, 0]]    
            else:
                #mode = 1
                if y_maxx < self.width:
                    self.alpha_line = [[self.keypoint[0],self.keypoint[1]], [0, y_maxx]]
                else:
                    self.alpha_line = [[self.keypoint[0],self.keypoint[1]], [x_maxy, 0]]
                
                if y_minx > 0:
                    self.beta_line = [[self.keypoint[0],self.keypoint[1]], [0, y_minx]]
                else:
                    self.beta_line = [[self.keypoint[0],self.keypoint[1]], [x_miny, 0]]   

        else:
            if center[1] <= self.keypoint[1]:
                #mode = 3
                if y_maxx < self.width:
                    self.alpha_line = [[self.keypoint[0],self.keypoint[1]], [0, y_maxx]]
                else:
                    self.alpha_line = [[self.keypoint[0],self.keypoint[1]], [x_maxy, 0]]
                
                if y_minx > 0:
                    self.beta_line = [[self.keypoint[0],self.keypoint[1]], [0, y_minx]]
                else:
                    self.beta_line = [[self.keypoint[0],self.keypoint[1]], [x_miny, 0]]   
            else:
                #mode = 2
                if y_maxx > 0:
                    self.alpha_line = [[self.keypoint[0],self.keypoint[1]], [0, y_maxx]]
                else:
                    self.alpha_line = [[self.keypoint[0],self.keypoint[1]], [x_maxy, 0]]
                
                if y_minx > 0:
                    self.beta_line = [[self.keypoint[0],self.keypoint[1]], [0, y_minx]]
                else:
                    self.beta_line = [[self.keypoint[0],self.keypoint[1]], [x_miny, 0]]           
    
    

    
class ReferenceFrameManager(object):
    
    def __init__(self, center_pos, obs_pos):
        self.center_pos = center_pos
        self.obs_pos = obs_pos
        self.k = float(self.obs_pos[0]-self.center_pos[0])/(self.obs_pos[1]-self.center_pos[1])
        #print self.k
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
            obs.initKeypoint()
            self.obstacles.append(obs)
            
            
            
        
        mX, mY = 0.0, 0.0
        for obs in self.obstacles:
            mX += obs.center[0]
            mY += obs.center[1]
        mX /= len(self.obstacles)
        mY /= len(self.obstacles)
        self.obsCenter = [int(mX), int(mY)]
        
        self.rf_mgrs = []
        for obs in self.obstacles:
            rf_mgr = ReferenceFrameManager(obs.keypoint, self.obsCenter)
            self.rf_mgrs.append(rf_mgr)
        
        
    def dump(self, filename):
        with open(filename, 'wb') as f:
            writer = csv.writer(f)
            writer.writerows(self.bin_data)
        
    def visualize(self):
        pygame.init()
        self.screen = pygame.display.set_mode((self.width,self.height))
        pygame.display.set_caption('Topological graph')
        self.screen.fill((255,255,255))
        
        for obs in self.obstacles:
            for o in obs.pixels:
                self.screen.set_at((o[0], o[1]), (122,122,122))
            pygame.draw.circle(self.screen, (255,0,0), [obs.keypoint[0],obs.keypoint[1]],3)
        
        pygame.draw.circle(self.screen, (0,0,0), [self.obsCenter[0], self.obsCenter[1]],3)
        
        while True:
            for event in pygame.event.get():
                if event.type == QUIT:
                    pygame.quit()
                    sys.exit()
            pygame.display.update()

        