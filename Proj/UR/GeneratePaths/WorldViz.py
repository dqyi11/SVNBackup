'''
Created on Jul 30, 2015

@author: daqing_yi
'''

import pygame, sys
from pygame.locals import *
import numpy as np
from Path import *

BLUE = (0,0,255)
RED = (255,0,0)
BLACK = (0,0,0)
GREEN = (0,255,0)

class WorldViz(object):

    def __init__(self, world):
        self.world = world
        
        pygame.init()
        self.screen = pygame.display.set_mode((int(self.world.width),int(self.world.height)))
        pygame.display.set_caption(self.world.name)
        self.screen.fill((255,255,255))
        
        self.colors = []
        for obj in self.world.objects:
            color = (np.random.randint(0,255), np.random.randint(0,255), np.random.randint(0,255))
            self.colors.append(color)
        
    def update(self):      
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    pos = pygame.mouse.get_pos()
                    print "LEFT " + str(pos)
                    self.world.init = pos
                else:
                    pos = pygame.mouse.get_pos()
                    print "RIGHT " + str(pos)
                    self.world.goal = pos
                    
        self.screen.fill((255,255,255))
            
        for i in range(len(self.world.objects)):
            obj = self.world.objects[i]
            pygame.draw.polygon(self.screen, self.colors[i], obj.polygon, 0)
            pygame.draw.circle(self.screen, BLACK, [int(obj.center.x), int(obj.center.y)], 1)
            
            pygame.draw.line(self.screen, GREEN, [int(obj.bounding[0]), int(obj.center.y)], [int(obj.bounding[2]),int(obj.center.y)], 2)
            pygame.draw.line(self.screen, GREEN, [int(obj.center.x), int(obj.bounding[1])], [int(obj.center.x), int(obj.bounding[3])], 2)

            
        
        if self.world.init != None:
            pygame.draw.circle(self.screen, BLUE, self.world.init, 10, 0)
        if self.world.goal != None:
            pygame.draw.circle(self.screen, RED, self.world.goal, 10, 0)
            
        pygame.display.flip()
        pygame.time.delay(100)
        
        return True
    
    def close(self):
        pygame.quit()
        
    def drawPath(self, path, filename):
        surface = pygame.Surface(self.world.width, self.world.height)
        surface.fill((255,255,255))
        for i in range(len(self.world.objects)):
            obj = self.world.objects[i]
            pygame.draw.polygon(self.screen, self.colors[i], obj.polygon, 0)
            pygame.draw.circle(self.screen, BLACK, [int(obj.center.x), int(obj.center.y)], 1)
            
            pygame.draw.line(self.screen, GREEN, [int(obj.bounding[0]), int(obj.center.y)], [int(obj.bounding[2]),int(obj.center.y)], 2)
            pygame.draw.line(self.screen, GREEN, [int(obj.center.x), int(obj.bounding[1])], [int(obj.center.x), int(obj.bounding[3])], 2)
            
        pathLen = len(path.waypoints)
        for i in range(pathLen-1):
            pygame.draw.line(self.screen, (255,255,0), path.waypoints[i], path.waypoints[i+1], 2)

        if self.world.init != None:
            pygame.draw.circle(self.screen, BLUE, self.world.init, 10, 0)
        if self.world.goal != None:
            pygame.draw.circle(self.screen, RED, self.world.goal, 10, 0)
        
        pygame.image.save(surface, filename)

        