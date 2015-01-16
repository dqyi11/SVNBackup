'''
Created on Jan 8, 2015

@author: daqing_yi
'''

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
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
                    self.scores[p,k] += self.costFuncs[k](pos_a, pos_b)
                    
    def visualize(self):
        
        if self.objectiveNum == 2:
            fig = plt.figure()     
            ax = fig.add_subplot(111)
            for i in range(self.pathNum):
                if i < self.objectiveNum:
                    ax.plot(self.scores[i,0], self.scores[i,1],'bs')
                else:
                    ax.plot(self.scores[i,0], self.scores[i,1],'ro')
            plt.show()
        elif self.objectiveNum == 3:
            fig1 = plt.figure()
            ax1 = fig1.add_subplot(111, projection='3d')
            ax1.scatter(self.scores[0,0], self.scores[1,1], self.scores[2,2], c='g', marker='p')
            for i in range(self.pathNum):
                if i < self.objectiveNum:
                    ax1.scatter(self.scores[i,0], self.scores[i,1], self.scores[i,2], c ='b', marker='s')
                else:
                    ax1.scatter(self.scores[i,0], self.scores[i,1], self.scores[i,2], c = 'r', marker='o')
            
            fig2 = plt.figure()     
            ax2 = fig2.add_subplot(111)
            for i in range(self.pathNum):
                if i == 0 or i == 1:
                    ax2.plot(self.scores[i,0], self.scores[i,1],'bs')
                else:
                    ax2.plot(self.scores[i,0], self.scores[i,1],'ro')
                    
            fig3 = plt.figure()     
            ax3 = fig3.add_subplot(111)
            for i in range(self.pathNum):
                if i == 0 or i == 2:
                    ax3.plot(self.scores[i,0], self.scores[i,2],'bs')
                else:
                    ax3.plot(self.scores[i,0], self.scores[i,2],'ro')
                    
            fig4 = plt.figure()     
            ax4 = fig4.add_subplot(111)
            for i in range(self.pathNum):
                if i == 1 or i == 2:
                    ax4.plot(self.scores[i,1], self.scores[i,2],'bs')
                else:
                    ax4.plot(self.scores[i,1], self.scores[i,2],'ro')
                    
            
            plt.show()
                
