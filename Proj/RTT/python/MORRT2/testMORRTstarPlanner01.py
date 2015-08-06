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
    
    def calcDist(currentPos, referencePos):
        dist = 0.0
        if referencePos==None:
            return dist
        dist = np.sqrt((currentPos[0]-referencePos[0])**2+(currentPos[1]-referencePos[1])**2)
        return dist
    
    MAP_FILE = './map2.png'
    
    planner = MORRTstarPlanner([600,400], 10, 2, [calcDist, calcCost], 50, MAP_FILE) 
    
    planner.morrts_viz.setName('MORRTstar01')
    planner.morrts_viz.loadObj([MAP_FILE, FIT_FILE])

    paths = planner.findPaths([40,40], [500, 40], 3000)
    print paths
    
    #planner.morrts_viz.saveResult()
    #planner.morrts_viz.saveResultInOne()
    
    #import pygame.image
    #pygame.image.save(planner.morrts_viz.screen, 'MORRTstar00.png')
    
    evaluator = MOPathEvaluator([calcDist, calcCost])
    evaluator.load(paths)
    
    
    evaluator.savePaths('MORRTstar01-path.txt')
    evaluator.saveScores('MORRTstar01-score.txt')
    evaluator.visualize()
    #np.savetxt('MORRTstar01-score.txt', evaluator.scores)
    
    print evaluator.scores
    
    while True:
        planner.morrts_viz.update()