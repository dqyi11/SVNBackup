'''
Created on Nov 3, 2014

@author: daqing_yi
'''

import pygame, sys
from pygame.locals import *
import numpy as np

class MORRTVisualizer(object):

    def __init__(self, morrt):
        self.morrt = morrt
        pygame.init()
        self.screen = pygame.display.set_mode((self.morrt.sampling_width,self.morrt.sampling_height))
        pygame.display.set_caption('MORRT')
        self.screen.fill((255,255,255))
        if self.morrt.mapfile != None:
            self.mapImg = pygame.image.load(self.morrt.mapfile)
        else:
            self.mapImg = None
        
        self.objImg = []
            
        self.activePaths = None
        self.dispMap = True
        self.totalIdx = self.morrt.objectiveNum + self.morrt.subproblemNum
        self.currIdx = 0
        self.currImgs = 0
        
    def loadObj(self, objFiles):    
        for obj in objFiles:
            self.objImgs.append(pygame.image.load(obj))

    def setLineColors(self, colors):
        self.lineColors = colors            
        
    def update(self):
        for e in pygame.event.get():
            if e.type == KEYDOWN:
                if e.key == pygame.K_UP:
                    self.currIdx += 1
                elif e.key == pygame.K_DOWN:
                    self.currImgs += 1
                elif e.key == pygame.K_RIGHT:
                    self.pathIdx += 1
            
            if self.currIdx >= self.totalIdx:
                self.currIdx = 0
            if self.currImgs >= len(self.objImg):
                self.currImgs = 0
            if self.pathIdx >= len(self.path):
                self.pathIdx = 0
            
        if self.objImg[self.currImgs] != None:
            self.screen.blit(self.objImg[self.currImgs],(0,0))
                
        disp_idx = 0
        if self.currIdx >= self.morrt.objectiveNum:
            disp_idx = self.currIdx - self.morrt.objectiveNum
        else:
            disp_idx = self.currIdx
        for n in self.morrt.referenceTrees[disp_idx].nodes:
            for c in n.children:
                pygame.draw.line(self.screen, (128,200,0), n.pos, c.pos)
                    
        pygame.draw.circle(self.screen, (255,0,0), self.morrt.start, 5)
        pygame.draw.circle(self.screen, (0,0,255), self.morrt.goal, 5)
        if self.morrt.new_pos != None and self.morrt.connected_pos != None:
            pygame.draw.line(self.screen, (200,128,0), self.morrt.new_pos, self.morrt.connected_pos)
            
        if self.activePaths != None:
            ap = self.activePaths[self.pathIdx]
            pathLen = len(ap)
            for i in range(0, pathLen-1, 1):
                pygame.draw.line(self.screen, (255, 160, 0), ap[i], ap[i+1], 2)
        
        '''
        for e in pygame.event.get():
            if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                sys.exit("Leaving because you requested it.")
        '''
        pygame.display.flip();
        #pygame.time.delay(200)
        