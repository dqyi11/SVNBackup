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
        
    def drawVertice(self, node):
        
        if node != None:
            if node.left!=None:
                pygame.draw.line(self.screen, (0,0,0), node.pos+self.offset, node.left.pos+self.offset)
                self.drawVertice(node.left)
            if node.right!=None:
                pygame.draw.line(self.screen, (0,0,0), node.pos+self.offset, node.right.pos+self.offset)
                self.drawVertice(node.right)
        
