'''
Created on Jan 23, 2015

@author: daqing_yi
'''

import cv2

from ObstacleMgr import *
import planar

import pygame, sys
from pygame.locals import *

class WorldMapMgr(object):


    def __init__(self):
        self.obstacles = []
        
    
    def load(self, mapfile):
        self.mapfile = mapfile
        self.mapImgData = cv2.imread(mapfile)
        self.height = self.mapImgData.shape[0]
        self.width = self.mapImgData.shape[1]
        self.mapImgGrayData = cv2.cvtColor(self.mapImgData, cv2.COLOR_BGR2GRAY)

        ret, thresh = cv2.threshold(255-self.mapImgGrayData, 127, 255, 0)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        for cont in contours:
            obs = ObstacleMgr(cont)
            self.obstacles.append(obs)
            
        self.init()
        
    def init(self):
        # select random point for each obstacle
        for obs in self.obstacles:
            obs.bk = obs.samplePosition()
            
        self.obsBkPairLines = []
        for i in range(len(self.obstacles)):
            for j in range(i, len(self.obstacles)):
                bk1 = self.obstacles[i].bk
                bk2 = self.obstacles[j].bk
                
                pline = planar.Line(planar.Vec2(bk1[0], bk1[1]),planar.Vec2(bk2[0], bk2[1]))
                self.obsBkPairLines.append(pline)
                
        
        # select central point c
        self.centralPoint = (int(self.width/2), int(self.height/2))
        foundCP = False
        while foundCP == False:
            if self.isInObsBkLinePair(self.centralPoint)==False:
                foundCP = True
            else:
                self.centralPoint[0] = np.random.randint(0, self.width)
                self.centralPoint[1] = np.random.randint(0, self.height)
        
    def isInObsBkLinePair(self, pos):
        
        pPos = planar.Point(pos[0], pos[1])
        for bkline in self.obsBkPairLines:
            if bkline.contains_point(pPos):
                return True
        return False    
        
    def visualize(self):
        pygame.init()
        self.screen = pygame.display.set_mode((self.width,self.height))
        pygame.display.set_caption('Topological graph')
        self.screen.fill((255,255,255))
        
        for obs in self.obstacles:
            xs, ys = obs.polygon.exterior.coords.xy
            for i in range(len(xs)-1):
                pygame.draw.line(self.screen, (122,122,122), (xs[i], ys[i]), (xs[i+1], ys[i+1]), 2)
            pygame.draw.circle(self.screen, (124,252,0), obs.bk, 4)
            
        pygame.draw.circle(self.screen, (255,0,0), self.centralPoint, 4)     
            
        while True:
            for event in pygame.event.get():
                if event.type == QUIT:
                    pygame.quit()
                    sys.exit()
            pygame.display.update()
            

