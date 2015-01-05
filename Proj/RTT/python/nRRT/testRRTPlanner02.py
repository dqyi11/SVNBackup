'''
Created on Jan 4, 2015

@author: daqing_yi
'''

from RRTPlanner import *
import time

if __name__ == '__main__':
    
    MAP_FILE = './map.png'
    
    planner = RRTPlanner([640, 400], 10, MAP_FILE) 

    path = planner.findPath([40,40], [500, 300], 1000)
    print path
    
    import pygame.image
    pygame.image.save(planner.rrt_viz.screen, 'RRT02.png')
    while True:
        time.sleep(5)