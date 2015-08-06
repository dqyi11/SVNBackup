'''
Created on Jan 4, 2015

@author: daqing_yi
'''

from RRTstarPlanner import *
import time
from scipy.misc import imread

if __name__ == '__main__':
    
    FIT_FILE = 'fitness1.png'
    objVals = np.array(imread(FIT_FILE, True))
    stepLen = 1
    
    
    def calcCost(currentPos, referencePos):
        cost = 0.0
        if referencePos==None:
            return cost
        
        x1 = int(currentPos[0])
        y1 = int(currentPos[1])
        x2 = int(referencePos[0])
        y2 = int(referencePos[1])
        
        dx = x2 - x1
        dy = y2 - y1
        is_steep = abs(dy) > abs(dx)
        if is_steep:
            x1, y1 = y1, x1
            x2, y2 = y2, x2
 
        swapped = False
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
            swapped = True
 
        dx = x2 - x1
        dy = y2 - y1
 
        error = int(dx / 2.0)
        ystep = 1 if y1 < y2 else -1
 
        y = y1
        points = []
        for x in range(x1, x2):
            coord = (y, x) if is_steep else (x, y)
            points.append(coord)
            if coord[1] >= objVals.shape[0] or coord[0] >= objVals.shape[1]:
                continue
            cost += objVals[int(coord[1]),int(coord[0])]/255.0
            error -= abs(dy)
            if error < 0:
                y += ystep
                error += dx
                
        return cost 
    
    planner = RRTstarPlanner([600,400], 10, calcCost) 
    
    planner.rrts_viz.loadObj(FIT_FILE)

    path = planner.findPath([40,40], [500, 40], 2000)
    print path
    
    import pygame.image
    pygame.image.save(planner.rrts_viz.screen, 'RRTstar00-1.png')
    
    while True:
        planner.rrts_viz.update()