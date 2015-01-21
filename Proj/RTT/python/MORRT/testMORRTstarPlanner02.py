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
    
    FIT_FILE1 = 'fitness01.jpg'
    FIT_FILE2 = 'fitness02.jpg'
    obj1Vals = np.array(imread(FIT_FILE1, True))
    obj2Vals = np.array(imread(FIT_FILE2, True))
    stepLen = 1
    
    def calcCost1(currentPos, referencePos):
        cost = 0.0
        if referencePos==None:
            return cost
        
        pos_a = currentPos
        pos_b = referencePos
        if pos_a[0] == pos_b[0] and pos_a[1] == pos_b[1]:
            return cost
        
        x_dist = np.abs(pos_a[0] - pos_b[0])
        y_dist = np.abs(pos_a[1] - pos_b[1])
        
        if x_dist > y_dist:
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
                if coordY >= obj1Vals.shape[0] or coordX >= obj1Vals.shape[1]: break
                cost += obj1Vals[int(coordY),int(coordX)]/255.0
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
                if coordY >= obj1Vals.shape[0] or coordX >= obj1Vals.shape[1]: break
                cost += obj1Vals[int(coordY),int(coordX)]/255.0

        return cost   
    
    def calcCost2(currentPos, referencePos):
        cost = 0.0
        if referencePos==None:
            return cost
        
        pos_a = currentPos
        pos_b = referencePos
        if pos_a[0] == pos_b[0] and pos_a[1] == pos_b[1]:
            return cost
        
        x_dist = np.abs(pos_a[0] - pos_b[0])
        y_dist = np.abs(pos_a[1] - pos_b[1])
        
        if x_dist > y_dist:
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
                if coordY >= obj2Vals.shape[0] or coordX >= obj2Vals.shape[1]: break
                cost += obj2Vals[int(coordY),int(coordX)]/255.0
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
                if coordY >= obj2Vals.shape[0] or coordX >= obj2Vals.shape[1]: break
                cost += obj2Vals[int(coordY),int(coordX)]/255.0

        return cost       
    
    def calcDist(currentPos, referencePos):
        dist = 0.0
        if referencePos==None:
            return dist
        dist = np.sqrt((currentPos[0]-referencePos[0])**2+(currentPos[1]-referencePos[1])**2)
        return dist   
    
    planner = MORRTstarPlanner([600,400], 10, 3, [calcDist, calcCost1, calcCost2], 100) 
    
    planner.morrts_viz.setName('MORRTstar02')
    planner.morrts_viz.loadObj([FIT_FILE1, FIT_FILE2])

    paths = planner.findPaths([40,40], [500, 40], 5000)
    print paths
    
    #import pygame.image
    #pygame.image.save(planner.morrts_viz.screen, 'MORRTstar00.png')
    
    planner.morrts_viz.saveResult()
    planner.morrts_viz.saveResultInOne()
    
    evaluator = MOPathEvaluator([calcDist, calcCost1, calcCost2])
    evaluator.load(paths)
    evaluator.visualize()
    
    evaluator.savePaths('MORRTstar02-path.txt')
    np.savetxt('MORRTstar02-score.txt', evaluator.scores)
        
    print evaluator.scores

    
    
    while True:
        planner.morrts_viz.update()