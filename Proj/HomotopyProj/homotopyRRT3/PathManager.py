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
        
        self.supportingClasses = []
        
        self.foundClasses = []
        
    def getString(self, stringBits):
        str_out = ""
        if len(stringBits) > 0:
            for i in range(len(stringBits)-1):
                str_out += str(stringBits[i])+","
            str_out += str(stringBits[len(stringBits)-1])
        return str_out    
    
    def loadSupportingClasses(self, classes):
        for cls in classes:
            cls_str = self.getString(cls)
            self.supportingClasses.append(cls_str)
        
    def importPath(self, path):
        
        str_out = self.getString(path.stringBits)
        if True:
        #if str_out in self.supportingClasses:
            if str_out in self.bestPaths.keys():
                if path.cost < self.bestPaths[str_out].cost:
                    self.bestPaths[str_out] = path
            else:
                self.bestPaths[str_out] = path
                self.foundClasses.append(path.stringBits)
            
    def getPaths(self):
        paths = []
        
        for kstr in self.bestPaths.keys():
            paths.append(self.bestPaths[kstr].points)
        
        return paths
    
    def getPathInfos(self):
        pathInfos = []
        for kstr in self.bestPaths.keys():
            pathInfos.append(self.bestPaths[kstr].points)
        return pathInfos

        
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
                f1.write( cstr + "\n" )
                
                p = self.bestPaths[cstr]
                f1.write(str(p.points) + "\n" )
                f1.write(str(p.stringBits) + "\n" )
                f1.write(str(p.cost) + "\n" )
                f1.write("\n")
                    
    def visualize(self):
        
        fig = plt.figure()
        ax = fig.add_subplot(111)
        index = np.arange(len(self.bestPaths.keys()))
        bar_width = 0.4
        scores = []
        for kstr in self.bestPaths.keys():
            scores.append(self.bestPaths[kstr].cost)
            
        ax.bar(index, scores, bar_width)    
        plt.show()
        
    def reportEquivalence(self, trajectoryReader):
        
        for i in range( len(self.foundClasses)-1 ):
            for j in range( i+1, len(self.foundClasses) ):
                
                refPathStr = self.foundClasses[i]
                pathStr = self.foundClasses[j]
                if trajectoryReader.compareStringPath(pathStr, refPathStr)==True:
                    print "Found Equivalence"
                    print self.getString(pathStr) 
                    print self.getString(refPathStr)
                