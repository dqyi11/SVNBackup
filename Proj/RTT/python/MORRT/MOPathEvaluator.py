'''
Created on Jan 8, 2015

@author: daqing_yi
'''

import matplotlib.pyplot as plt
import numpy as np

class MOPathEvaluator(object):

    def __init__(self, cost_funcs):
        self.objectiveNum = len(cost_funcs)
        self.costFuncs = cost_funcs
        self.paths = []
        
    def load(self, paths):
        self.paths = paths
        self.pathNum = len(paths)
        
        self.scores = np.zeros((self.pathNum, self.objectiveNum))
        for p in range(self.pathNum):
            path = self.paths[p]
            pathLen = len(path)
            for i in range(pathLen-1):
                pos_a = path[i]
                pos_b = path[i+1]
                for k in range(self.objectiveNum):
                    self.scores[i,k] += self.costFuncs[k](pos_a, pos_b)
                    
    def visualize(self):
    
        fig = plt.figure()     
        ax = fig.add_subplot(111)
        for i in range(self.pathNum):
            if i < self.objectiveNum:
                ax.plot(self.scores[i,0], self.scores[i,1],'bs')
            else:
                ax.plot(self.scores[i,0], self.scores[i,1],'r.')
        plt.show()
                
