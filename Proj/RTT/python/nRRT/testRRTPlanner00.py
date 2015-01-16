'''
Created on Jan 4, 2015

@author: daqing_yi
'''

from RRTPlanner import *
import time

if __name__ == '__main__':
    

    planner = RRTPlanner([600,400], 10) 

    path = planner.findPath([40,40], [320, 300], 1000)
    print path
    
    import pygame.image
    pygame.image.save(planner.rrt_viz.screen, 'RRT00.png')
    
    while True:
        planner.rrt_viz.update()

    
