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
            
        
    def update(self):
        if self.mapImg != None:
            self.screen.blit(self.mapImg,(0,0))
        for n in self.rrt.nodes:
            for c in n.children:
                #print str(n.pos) + "-" + str(c.pos)
                pygame.draw.line(self.screen, (128,200,0), n.pos, c.pos)
        pygame.draw.circle(self.screen, (255,0,0), self.rrt.start, 5)
        pygame.draw.circle(self.screen, (0,0,255), self.rrt.goal, 5)
        if self.rrt.new_node != None and self.rrt.connected_node != None:
            pygame.draw.line(self.screen, (200,128,0), self.rrt.new_node, self.rrt.connected_node)
        
        #pygame.display.update()
        '''
        for e in pygame.event.get():
            if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                sys.exit("Leaving because you requested it.")
        '''
        pygame.display.flip();
        pygame.time.delay(200)
        