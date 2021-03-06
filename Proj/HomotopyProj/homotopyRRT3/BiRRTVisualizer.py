'''
Created on Nov 3, 2014

@author: daqing_yi
'''

import pygame, sys
from pygame.locals import *
import numpy as np

class BiRRTVisualizer(object):

    def __init__(self, rrt, pathMgr, name=None):
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
        self.name = name
            
        self.activePaths = []
        self.dispMap = True
        
        self.pathMgr = pathMgr
        self.refLines = []
        self.ref_subregs = []
        
        self.states = ("START", "GOAL", "BOTH")
        self.currentState = 2
        
        self.currentPaths = []
        
        self.font = pygame.font.SysFont(None, 16)
        
        self.pathIdx = -1
        
        self.displayRefFrames = False

        
    def loadObj(self, objFile):    
        self.objImg = pygame.image.load(objFile)
        
    def update(self):
        
        for e in pygame.event.get():
            if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                sys.exit("Quit it.")
            if e.type == KEYDOWN:
                if e.key == pygame.K_m:
                    if self.dispMap==True:
                        self.dispMap = False
                    else:
                        self.dispMap = True
                elif e.key == pygame.K_LEFT:
                    self.pathIdx -= 1
                elif e.key == pygame.K_RIGHT:
                    self.pathIdx += 1
                elif e.key == pygame.K_s:
                    if self.displayRefFrames == True:
                        self.displayRefFrames = False
                    else:
                        self.displayRefFrames = True
                        
                elif e.key == pygame.K_g:
                    self.pathMgr.mergeBestPaths(self.rrt.homotopyMgr.reader)
                    self.activePaths = self.pathMgr.getPaths()
                    
                    self.pathMgr.visualize()
                    self.pathMgr.savePaths(self.name+"-merged.txt")
                    
        if self.pathIdx >= len(self.pathMgr.bestPaths.keys()):
            self.pathIdx = -1
        elif self.pathIdx < -1:
            self.pathIdx = len(self.pathMgr.bestPaths.keys())-1
            
        self.screen.fill((255,255,255))
        
        if self.dispMap==True:
            if self.mapImg != None:
                self.screen.blit(self.mapImg,(0,0))
        else:
            if self.objImg != None:
                self.screen.blit(self.objImg,(0,0))
                
        for refLine in self.refLines:
            pygame.draw.line(self.screen, (50,50,50), refLine[0][0], refLine[0][1])
            if self.displayRefFrames == True:
                self.screen.blit(self.font.render(refLine[1], True, (0,255,0)), refLine[2])
                
        if self.displayRefFrames == True:
            for subreg in self.ref_subregs:
                self.screen.blit(self.font.render(subreg[0], True, (0,0,255)), subreg[1])
        '''
        for dr in self.rrt.dividingRefs:
            pygame.draw.line(self.screen, (255,204,153), dr[0], dr[1], 10)
        '''
         
        if self.displayRefFrames == False:
            
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
                
        if len(self.currentPaths) > 0:
            for path in self.currentPaths:
                pathLen = len(path)
                for i in range(0, pathLen-1, 1):
                    pygame.draw.line(self.screen, (255, 0, 0), path[i], path[i+1], 2)        
                
        if len(self.pathMgr.bestPaths.keys()) > 0 and self.pathIdx >= 0:
            kstr = self.pathMgr.bestPaths.keys()[self.pathIdx]
            activePath = self.pathMgr.bestPaths[kstr].points
            pathLen = len(activePath)
            for i in range(0, pathLen-1, 1):
                pygame.draw.line(self.screen, (0, 102, 204), activePath[i], activePath[i+1], 2)  
            self.screen.blit(self.font.render(kstr, True, (255,0,0)), (10,10))
                
                
        start = (int(self.rrt.start[0]), int(self.rrt.start[1]))
        goal = (int(self.rrt.goal[0]), int(self.rrt.goal[1]))
        pygame.draw.circle(self.screen, (255,0,0), start, 5)
        pygame.draw.circle(self.screen, (0,0,255), goal, 5)        
        
        
                
        pygame.display.flip();

        