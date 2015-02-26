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
        
        self.classes = {}
        self.bestPaths = {}
        
    def getString(self, stringBits):
        str_out = ""
        if len(stringBits) > 0:
            for i in range(len(stringBits)-1):
                str_out += str(stringBits[i])+"-"
            str_out += str(stringBits[len(stringBits)-1])
        return str_out    
        
    def importPath(self, path):
        
        str_out = self.getString(path.stringBits)
        if str_out in self.bestPaths.keys():
            if path.cost < self.bestPaths[str_out]:
                self.bestPaths[str_out] = path
        else:
            self.bestPaths[str_out] = path

        
    def getClass(self, strBit):
        for cstr in self.classes.keys():
            same = True
            if len(cstr)==len(strBit):
                for i in range(len(cstr)):
                    if cstr[i]!=strBit[i]:
                        same = False
            if same==True:
                return self.classes[cstr]
        return None

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
            f1.write( "Total Num " + str(len(self.bestPaths.keys())) + "\n")
            for cstr in self.bestPaths.keys():
                f1.write( str(len(self.classes[cstr])) + " : " + str(cstr) + "\n" )
                
                p = self.bestPaths[cstr]
                f1.write(str(p.points))
                f1.write(str(p.stringBits))
                f1.write(str(p.cost))
                f1.write("\n")
                    
    def visualize(self):
        
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.plot([], [])
        plt.show()