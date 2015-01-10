'''
Created on Jan 4, 2015

@author: daqing_yi
'''

from kRRTstarPlanner import *
import time

if __name__ == '__main__':
    
    def calcDist(currentPos, referencePos):
        dist = 0.0
        if referencePos==None:
            return dist
        dist = np.sqrt((currentPos[0]-referencePos[0])**2+(currentPos[1]-referencePos[1])**2)
        return dist   
    
    planner = kRRTstarPlanner([600,400], 10, calcDist) 

    path = planner.findPath([40,40], [500, 40], 1000)
    print path
    
    import pygame.image
    pygame.image.save(planner.krrts_viz.screen, 'kRRTstar00.png')
    
    while True:
        time.sleep(5)