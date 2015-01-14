'''
Created on Jan 4, 2015

@author: daqing_yi
'''

from RRGPlanner import *
import time

if __name__ == '__main__':
    
    def calcDist(currentNode, referenceNode):
        dist = 0.0
        if referenceNode==None:
            return dist
        dist = np.sqrt((currentNode.pos[0]-referenceNode.pos[0])**2+(currentNode.pos[1]-referenceNode.pos[1])**2)
        return dist   
    
        
    MAP_FILE = './lab-map-inferior.png'
    MAP_FILE = './lab-map-scaled.png'
    
    planner = RRGPlanner([444, 989], 10, calcDist, MAP_FILE) 

    path = planner.findPath([40,40], [320, 300], 2000)
    print path
    
    import pygame.image
    pygame.image.save(planner.rrg_viz.screen, 'RRG01.png')
    
    while True:
        planner.rrg_viz.update()