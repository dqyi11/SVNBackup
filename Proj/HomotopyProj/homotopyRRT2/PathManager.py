'''
Created on Jan 8, 2015

@author: daqing_yi
'''

import matplotlib.pyplot as plt
import numpy as np

class Path(object):
    
    def __init__(self, points, cost, stringBits):
        self.points = points
        self.cost = cost
        self.stringBits = stringBits

class PathManager(object):

    def __init__(self, cost_func):

        self.costFunc = cost_func
        self.paths = []
        
    def addPath(self, path):
        self.paths.append(path)
        self.pathNum = len(self.paths)
        
        
    def calcCost(self, path): 
        score = 0.0
        pathLen = len(path)
        for i in range(pathLen-1):
            pos_a = path[i]
            pos_b = path[i+1]
            score += self.costFunc(pos_a, pos_b)
        return score
                    
    def savePaths(self, filename):
        with open(filename, 'w') as f1:
            for p in self.paths:
                f1.write(str(p.points))
                f1.write(str(p.stringBits))
                f1.write(str(p.cost))
                f1.write("\n")
                    
    def visualize(self):
        
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.plot([], [])
        plt.show()