'''
Created on Nov 3, 2014

@author: daqing_yi
'''

import pygame, sys
from pygame.locals import *
import numpy as np

class RRTVisualizer(object):

    def __init__(self, rrt):
        self.rrt = rrt
        pygame.init()
        self.screen = pygame.display.set_mode((self.rrt.sampling_width,self.rrt.sampling_height))
        pygame.display.set_caption('RRT')
        self.screen.fill((255,255,255))
        if self.rrt.mapfile != None:
            self.mapImg = pygame.image.load(self.rrt.mapfile)
        else:
            self.mapImg = None
        
        self.objImg = None
            
        self.activePath = None
        self.dispMap = True
        
    def loadObj(self, objFile):    
        self.objImg = pygame.image.load(objFile)
            
        
    def update(self):
        
        for e in pygame.event.get():
            if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                sys.exit("Leaving because you requested it.")
            elif e.type == KEYDOWN:
                if e.key == pygame.K_UP:
                    if self.dispMap==True:
                        self.dispMap = False
                    else:
                        self.dispMap = True
            
        self.screen.fill((255,255,255))    
        if self.dispMap==True:
            if self.mapImg != None:
                self.screen.blit(self.mapImg,(0,0))
        else:
            if self.objImg != None:
                self.screen.blit(self.objImg,(0,0))
                
        for n in self.rrt.nodes:
            for c in n.children:
                #print str(n.pos) + "-" + str(c.pos)
                pygame.draw.line(self.screen, (128,200,0), n.pos, c.pos)
        pygame.draw.circle(self.screen, (255,0,0), self.rrt.start, 5)
        pygame.draw.circle(self.screen, (0,0,255), self.rrt.goal, 5)
        if self.rrt.new_pos != None and self.rrt.connected_pos != None:
            pygame.draw.line(self.screen, (200,128,0), self.rrt.new_pos, self.rrt.connected_pos)
            
        if self.activePath != None:
            pathLen = len(self.activePath)
            for i in range(0, pathLen-1, 1):
                pygame.draw.line(self.screen, (255, 160, 0), self.activePath[i], self.activePath[i+1], 2)
        
        #pygame.display.update()
        '''
        for e in pygame.event.get():
            if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                sys.exit("Leaving because you requested it.")
        '''
        pygame.display.flip();
        #pygame.time.delay(200)
        