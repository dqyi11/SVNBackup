'''
Created on Jan 4, 2015

@author: daqing_yi
'''

from MORRTstarPlanner import *
from scipy.misc import imread
import time
import numpy as np
from MOPathEvaluator import *

if __name__ == '__main__':
    
    CFIT_FILE1 = 'cfitness1.png'
    CFIT_FILE2 = 'cfitness2.png'
    objVals1 = np.array(imread(CFIT_FILE1, True))
    objVals2 = np.array(imread(CFIT_FILE2, True))
    stepLen = 1
    
    
    def calcCost1(currentPos, referencePos):
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
            if coord[1] >= objVals1.shape[0] or coord[0] >= objVals1.shape[1]:
                continue
            cost += objVals1[int(coord[1]),int(coord[0])]/255.0
            error -= abs(dy)
            if error < 0:
                y += ystep
                error += dx
                
        return cost
    
    def calcCost2(currentPos, referencePos):
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
            if coord[1] >= objVals2.shape[0] or coord[0] >= objVals2.shape[1]:
                continue
            cost += objVals2[int(coord[1]),int(coord[0])]/255.0
            error -= abs(dy)
            if error < 0:
                y += ystep
                error += dx
                
        return cost
    
    planner = MORRTstarPlanner([600,400], 10, 2, [calcCost1, calcCost2], 30) 
    
    planner.morrts_viz.setName('MORRTstar03')
    planner.morrts_viz.loadObj([CFIT_FILE1, CFIT_FILE2])

    paths = planner.findPaths([40,200], [550, 200], 5000)
    print paths
    
    planner.morrts_viz.saveResult()
    planner.morrts_viz.saveResultInOne()
    
    #import pygame.image
    #pygame.image.save(planner.morrts_viz.screen, 'MORRTstar00.png')
    
    evaluator = MOPathEvaluator([calcCost1, calcCost2])
    evaluator.load(paths)
    evaluator.visualize()
    
    evaluator.savePaths('MORRTstar03-path.txt')
    np.savetxt('MORRTstar03-score.txt', evaluator.scores)
    
    print evaluator.scores
    
    while True:
        planner.morrts_viz.update()