'''
Created on Jan 15, 2015

@author: daqing_yi
'''

from MOPathPlanner import *
from FitnessManager import *
from MOPathEvaluator import *
from MOPathVisualizer import *
from scipy.misc import imread
from PyQt4 import QtGui, QtCore

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
    
    def calcDist(currentPos, referencePos):
        dist = 0.0
        if referencePos==None:
            return dist
        dist = np.sqrt((currentPos[0]-referencePos[0])**2+(currentPos[1]-referencePos[1])**2)
        return dist   
    
    world_size = [600,400]
    start = [40, 40]
    goal = [500, 40]
    sample_num = 20
    subproblem_num = 100
    iteration_num = 8000
    pos_range = []
    for i in range(sample_num):
        pos_range.append([0, world_size[0]-1])
        pos_range.append([0, world_size[1]-1])
        
    
    #fitMgr = FitnessManager()
    #fitMgr.addFitness(FIT_FILE)
    planner = MOPathPlanner([calcDist, calcCost],start, goal, sample_num) 
    

    paths = planner.findSolutions(subproblem_num, iteration_num, pos_range)
    print paths
    
    np.savetxt('MOPath01-path.txt', paths)
    
    viz = MOPathVisualizer(world_size, start, goal, "MOPath01")
    viz.loadPaths(paths)
    viz.loadObj([FIT_FILE])
    viz.saveResult()
    
    
    evaluator = MOPathEvaluator([calcDist, calcCost])
    evaluator.load(paths)
    evaluator.visualize()
    
    print evaluator.scores
    
    np.savetxt('MOPath01-score.txt', evaluator.scores)
    
    while True:
        viz.update()