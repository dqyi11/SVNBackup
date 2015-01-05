'''
Created on Jan 4, 2015

@author: daqing_yi
'''

from RRTPlanner import *
import time

if __name__ == '__main__':
    
    MAP_FILE = './lab-map-inferior.png'
    MAP_FILE = './lab-map-scaled.png'
    
    planner = RRTPlanner([444, 989], 10, MAP_FILE) 

    path = planner.findPath([40,40], [320, 300], 1000)
    print path
    
    import pygame.image
    pygame.image.save(planner.rrt_viz.screen, 'RRT01.png')
    while True:
        time.sleep(5)