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
        self.screen = pygame.display.set_mode((self.rrt.sampling_range[0][1]-self.rrt.sampling_range[0][0],self.rrt.sampling_range[1][1]-self.rrt.sampling_range[1][0]))
        self.offset = np.array([self.rrt.sampling_range[0][0], self.rrt.sampling_range[1][0]])
        pygame.display.set_caption('RRT')
        self.screen.fill((255,255,255))
        
    def update(self):
        for n in self.rrt.nodes:
            for c in n.children:
                #print str(n.pos) + "-" + str(c.pos)
                pygame.draw.line(self.screen, (0,0,0), n.pos+self.offset, c.pos+self.offset)
        pygame.draw.circle(self.screen, (255,0,0), self.rrt.start+self.offset, 5)
        pygame.draw.circle(self.screen, (0,0,255), self.rrt.goal+self.offset, 5)
        
        #pygame.display.update()
        '''
        for e in pygame.event.get():
            if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                sys.exit("Leaving because you requested it.")
        '''
        pygame.display.flip();
        #pygame.time.delay(100)
        