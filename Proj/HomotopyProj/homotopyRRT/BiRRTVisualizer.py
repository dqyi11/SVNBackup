'''
Created on Nov 3, 2014

@author: daqing_yi
'''

import pygame, sys
from pygame.locals import *
import numpy as np

class BiRRTVisualizer(object):

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
        
        self.refLines = []
        
        self.states = ("START", "GOAL", "BOTH")
        self.currentState = 2
        
    def loadObj(self, objFile):    
        self.objImg = pygame.image.load(objFile)
        
    def update(self):
        
        for e in pygame.event.get():
            if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                sys.exit("Quit it.")
            if e.type == KEYDOWN:
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
                
        for refLine in self.refLines:
            pygame.draw.line(self.screen, (50,50,50), refLine[0], refLine[1])
         
         
        if self.currentState==0 or self.currentState==2:       
            for n in self.rrt.st_nodes:
                for c in n.children:
                    #print str(n.pos) + "-" + str(c.pos)
                    n_pos = (int(n.pos[0]), int(n.pos[1]))
                    c_pos = (int(c.pos[0]), int(c.pos[1]))
                    pygame.draw.line(self.screen, (128,200,0), n_pos, c_pos)

            if self.rrt.st_new_node != None and self.rrt.st_connected_node != None:
                new_node = (int(self.rrt.st_new_node[0]), int(self.rrt.st_new_node[1]))
                connected_node = (int(self.rrt.st_connected_node[0]), int(self.rrt.st_connected_node[1]))
                pygame.draw.line(self.screen, (200,128,0), new_node, connected_node)
                
        
        if self.currentState==1 or self.currentState==2:
            for n in self.rrt.gt_nodes:
                for c in n.children:
                    #print str(n.pos) + "-" + str(c.pos)
                    n_pos = (int(n.pos[0]), int(n.pos[1]))
                    c_pos = (int(c.pos[0]), int(c.pos[1]))
                    pygame.draw.line(self.screen, (200,128,0), n_pos, c_pos)

            if self.rrt.gt_new_node != None and self.rrt.gt_connected_node != None:
                new_node = (int(self.rrt.gt_new_node[0]), int(self.rrt.gt_new_node[1]))
                connected_node = (int(self.rrt.gt_connected_node[0]), int(self.rrt.gt_connected_node[1]))
                pygame.draw.line(self.screen, (200,128,0), new_node, connected_node)
                
        start = (int(self.rrt.start[0]), int(self.rrt.start[1]))
        goal = (int(self.rrt.goal[0]), int(self.rrt.goal[1]))
        pygame.draw.circle(self.screen, (255,0,0), start, 5)
        pygame.draw.circle(self.screen, (0,0,255), goal, 5)        
                
        if self.activePath != None:
            pathLen = len(self.activePath)
            for i in range(0, pathLen-1, 1):
                pygame.draw.line(self.screen, (255, 160, 0), self.activePath[i], self.activePath[i+1], 2)            
        
        pygame.display.flip();

        