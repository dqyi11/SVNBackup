'''
Created on Nov 3, 2014

@author: daqing_yi
'''

import pygame, sys
from pygame.locals import *

class RRTVisualizer(object):

    def __init__(self, rrt):
        self.rrt = rrt
        pygame.init()
        self.screen = pygame.display.set_mode([self.rrt.dimension[0],self.rrt.dimension[1]])
        self.screen.fill((255,255,255))
        
    def update(self):
        for n in self.rrt.nodes:
            for c in n.children:
                pygame.draw.line(self.screen, (0,0,0), n.pos, c.pos)
        
        pygame.display.update()

        for e in pygame.event.get():
            if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                sys.exit("Leaving because you requested it.")
        