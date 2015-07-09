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
        
        pos_a = currentPos
        pos_b = referencePos
        if pos_a[0] == pos_b[0] and pos_a[1] == pos_b[1]:
            return cost
        
        x_dist = pos_a[0] - pos_b[0]
        y_dist = pos_a[1] - pos_b[1]
        
        abs_x_dist = np.abs(x_dist)
        abs_y_dist = np.abs(y_dist)
        
        if abs_x_dist > abs_y_dist:
            k = y_dist/x_dist
            if pos_a[0] < pos_b[0]:
                startX = pos_a[0]
                endX = pos_b[0]
                startY = pos_a[1]
            else:
                startX = pos_b[0]
                endX = pos_a[0]
                startY = pos_b[1]
            
            for coordX in np.arange(startX, endX, stepLen):
                coordY = int(k*(coordX-startX) + startY)
                if coordY >= objVals.shape[0] or coordX >= objVals.shape[1]: break
                cost += objVals[int(coordY),int(coordX)]/255.0
        else:
            k = x_dist/y_dist
            if pos_a[1] < pos_b[1]:
                startY = pos_a[1]
                endY = pos_b[1]
                startX = pos_a[0]
            else:
                startY = pos_b[1]
                endY = pos_a[1]
                startX = pos_b[0]
                
            for coordY in np.arange(startY, endY, stepLen):
                coordX = int(k*(coordY-startY) + startX)
                if coordY >= objVals.shape[0] or coordX >= objVals.shape[1]: break
                cost += objVals[int(coordY),int(coordX)]/255.0

        return cost   
    
    planner = RRTstarPlanner([600,400], 10, calcCost) 
    
    planner.rrts_viz.loadObj(FIT_FILE)

    path = planner.findPath([40,40], [500, 40], 2000)
    print path
    
    import pygame.image
    pygame.image.save(planner.rrts_viz.screen, 'RRTstar00-1.png')
    
    while True:
        planner.rrts_viz.update()