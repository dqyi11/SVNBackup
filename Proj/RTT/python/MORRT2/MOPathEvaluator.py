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
                    
    def savePaths(self, filename):
        
        with open(filename, 'w') as f1:
            for p in self.paths:
                for s in p:
                    f1.write(str(s[0])+" "+str(s[1]))
                    f1.write('\t')
                f1.write('\n')  
                
    def saveScores(self, filename):
        
        with open(filename, 'w') as f1:
            for p in self.scores:
                for k in range(self.objectiveNum):
                    f1.write(str(p[k]))
                    f1.write('\t')
                f1.write('\n')    
                    
    def visualize(self):
        
        if self.objectiveNum == 2:
            fig = plt.figure()     
            ax = fig.add_subplot(111)
            for i in range(self.pathNum):
                if i < self.objectiveNum:
                    ax.plot(self.scores[i,0], self.scores[i,1],'bs')
                else:
                    ax.plot(self.scores[i,0], self.scores[i,1],'ro')
                    
            ax.set_xlabel("objective 1")
            ax.set_ylabel("objective 2")
            plt.show()
        elif self.objectiveNum == 3:
            fig1 = plt.figure()
            ax1 = fig1.add_subplot(111, projection='3d')
            
            # interpolation spline with scipy 
            from scipy import interpolate 
            
            tck0 = interpolate.bisplrep(self.scores[:,0], self.scores[:,1], self.scores[:,2]) 
            xmin = np.min(self.scores[:,0])
            xmax = np.max(self.scores[:,0])
            ymin = np.min(self.scores[:,1])
            ymax = np.max(self.scores[:,1])
            xnew,ynew = np.mgrid[xmin:xmax:70j,ymin:ymax:70j] 
            znew = interpolate.bisplev(xnew[:,0],ynew[0,:],tck0) 
            ax1.plot_surface(xnew, ynew, znew, linewidth=0.0, color='y', alpha = 0.25)
            
            ax1.scatter(self.scores[0,0], self.scores[1,1], self.scores[2,2], c='g', marker='p')
            for i in range(self.pathNum):
                if i < self.objectiveNum:
                    ax1.scatter(self.scores[i,0], self.scores[i,1], self.scores[i,2], c ='b', marker='s')
                else:
                    ax1.scatter(self.scores[i,0], self.scores[i,1], self.scores[i,2], c = 'r', marker='o')
                    
            ax1.set_xlabel('Objective 1')
            ax1.set_ylabel('Objective 2')
            ax1.set_zlabel('Objective 3')
            
            '''
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
            '''        
            
            plt.show()
                
