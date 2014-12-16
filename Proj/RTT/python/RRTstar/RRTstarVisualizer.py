'''
Created on Dec 14, 2014

@author: daqing_yi
'''

import pygame, sys
from pygame.locals import *
import numpy as np

class RRTstarVisualizer(object):


    def __init__(self, rrt, dim_range):
        self.rrt = rrt 
        self.dim_range = dim_range
        self.width = dim_range[0][1] - dim_range[0][0]
        self.height = dim_range[1][1] - dim_range[1][0]
        self.offset = np.zeros(2)
        self.offset[0] = dim_range[0][0]
        self.offset[1] = dim_range[1][0]
        
        pygame.init()
        self.screen = pygame.display.set_mode((int(self.width),int(self.height)))
        pygame.display.set_caption('RRT')
        self.screen.fill((255,255,255))
        
    
    def update(self):      
        self.drawVertice(self.rrt.root)
        pygame.display.flip();
        pygame.time.delay(100)
        
    def drawVertice(self, node):
        
        if node != None:
            if node.left!=None:
                from_pos = node.pos-self.offset
                to_pos = node.left.pos-self.offset
                #print "PLOT " + str(from_pos[0]) +"," + str(from_pos[1]) + " - " + str(to_pos[0]) + "," + str(to_pos[1])
                pygame.draw.line(self.screen, (0,0,0), from_pos , to_pos)
                self.drawVertice(node.left)
            if node.right!=None:
                from_pos = node.pos-self.offset
                to_pos = node.right.pos-self.offset
                #print "PLOT " + str(from_pos[0]) +"," + str(from_pos[1]) + " - " + str(to_pos[0]) + "," + str(to_pos[1])
                pygame.draw.line(self.screen, (0,0,0), from_pos , to_pos)
                self.drawVertice(node.right)
        
