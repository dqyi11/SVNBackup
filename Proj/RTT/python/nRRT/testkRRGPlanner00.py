'''
Created on Jan 4, 2015

@author: daqing_yi
'''

from kRRGPlanner import *
import time

if __name__ == '__main__':
    
    def calcDist(currentNode, referenceNode):
        dist = 0.0
        if referenceNode==None:
            return dist
        dist = np.sqrt((currentNode.pos[0]-referenceNode.pos[0])**2+(currentNode.pos[1]-referenceNode.pos[1])**2)
        return dist   
    
    planner = kRRGPlanner([600,400], 10, calcDist) 

    path = planner.findPath([40,40], [500, 40], 1000)
    print path
    
    import pygame.image
    pygame.image.save(planner.krrg_viz.screen, 'kRRG00.png')
    
    while True:
        planner.krrg_viz.update()