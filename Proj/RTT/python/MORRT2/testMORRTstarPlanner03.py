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
                if coordY >= objVals1.shape[0] or coordX >= objVals1.shape[1]: break
                cost += objVals1[int(coordY),int(coordX)]/255.0
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
                if coordY >= objVals1.shape[0] or coordX >= objVals1.shape[1]: break
                cost += objVals1[int(coordY),int(coordX)]/255.0

        return cost   
    
    def calcCost2(currentPos, referencePos):
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
                if coordY >= objVals2.shape[0] or coordX >= objVals2.shape[1]: break
                cost += objVals2[int(coordY),int(coordX)]/255.0
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
                if coordY >= objVals2.shape[0] or coordX >= objVals2.shape[1]: break
                cost += objVals2[int(coordY),int(coordX)]/255.0

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