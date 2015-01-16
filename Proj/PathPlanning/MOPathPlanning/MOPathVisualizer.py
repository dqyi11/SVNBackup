'''
Created on Jan 15, 2015

@author: daqing_yi
'''

import pygame, sys
from pygame.locals import *
import pygame.image
import numpy as np

class MOPathVisualizer(object):

    def __init__(self, world_range, start, goal, name=""):
        
        pygame.init()
        self.world_range = world_range
        self.screen = pygame.display.set_mode((world_range[0],world_range[1]))
        self.screen.fill((255,255,255))
        
        self.start = start
        self.goal = goal

        self.objImgs = []
            
        self.activePaths = []
        self.dispMap = True
 
        self.pathIdx = 0
        self.currImgs = 0
        
        self.font = pygame.font.SysFont(None, 24)
        
    def setName(self, name):
        self.name = name
    
    def loadPaths(self, paths):
        self.activePaths = paths
        
    def loadObj(self, objFiles):    
        for obj in objFiles:
            self.objImgs.append(pygame.image.load(obj))

    def setLineColors(self, colors):
        self.lineColors = colors    
        
    def saveResult(self):
        for idx in range(self.totalIdx):
            self.screen.fill((255,255,255))
                        
            ap = self.activePaths[idx]
            pathLen = len(ap)
            for i in range(0, pathLen-1, 1):
                app = (int(ap[i][0]), int(ap[i][1]))
                app_n = (int(ap[i+1][0]), int(ap[i+1][1]))
                pygame.draw.line(self.screen, (255, 160, 0), app, app_n, 2)
                
            start = (int(self.start[0]), int(self.start[1]))
            goal = (int(self.goal[0]), int(self.goal[1]))
            pygame.draw.circle(self.screen, (255,0,0), start, 5)
            pygame.draw.circle(self.screen, (0,0,255), goal, 5)
                
            pygame.image.save(self.screen, './fig/'+self.name+"-"+str(idx)+".png")
 
            
            
        
    def update(self):
        for e in pygame.event.get():
            if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                sys.exit("Quit it.")
            elif e.type == KEYDOWN:

                if e.key == pygame.K_LEFT:
                    self.pathIdx -= 1
                elif e.key == pygame.K_RIGHT:
                    self.pathIdx += 1
                elif e.key == pygame.K_p:
                    self.currImgs += 1
                elif e.key == pygame.K_o:
                    self.currImgs -= 1
                elif e.key == pygame.K_s:
                    pygame.image.save(self.screen, self.name+"-"+str(self.pathIdx)+".png")
        
            if self.currImgs < 0:
                self.currImgs = len(self.objImgs)-1
            elif self.currImgs >= len(self.objImgs):
                self.currImgs = 0
            if self.pathIdx < 0:
                self.pathIdx = len(self.activePaths)-1
            elif self.pathIdx >= len(self.activePaths):
                self.pathIdx = 0        
        
        self.screen.fill((255,255,255))       
        if self.objImgs[self.currImgs] != None:
            self.screen.blit(self.objImgs[self.currImgs],(0,0))

        start = (int(self.start[0]), int(self.start[1]))
        goal = (int(self.goal[0]), int(self.goal[1]))
        pygame.draw.circle(self.screen, (255,0,0), start, 5)
        pygame.draw.circle(self.screen, (0,0,255), goal, 5)
 
        if len(self.activePaths) > 0:
            ap = self.activePaths[self.pathIdx]
            pathLen = len(ap)
            for i in range(0, pathLen-1, 1):
                app = (int(ap[i][0]), int(ap[i][1]))
                app_n = (int(ap[i+1][0]), int(ap[i+1][1]))
                pygame.draw.line(self.screen, (255, 160, 0), app, app_n, 2)
        
        '''
        for e in pygame.event.get():
            if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                sys.exit("Leaving because you requested it.")
        '''
        self.screen.blit(self.font.render("PI:"+str(self.pathIdx), True, (255,0,0)), (self.world_range[0]-40, 10))
        #self.screen.blit(self.font.render("TI:"+str(self.currIdx), True, (0,255,0)), (self.morrt.sampling_width-40, 40))
                
        pygame.display.flip();
        #pygame.time.delay(200)
        