'''
Created on Oct 29, 2014

@author: daqing_yi
'''

import numpy as np
from UnionFindSet import *

neighbor_operators = [[-1, -1], [-1, 0], [-1, 1], [0, -1]]

class ConnectedComponentMgr(object):

    def __init__(self, data):
        
        uf = UnionFindSet()
        self.labelData = - np.ones(data.shape, np.int)
        self.width = data.shape[0]
        self.height = data.shape[1]
        self.data = data
        self.maxLabel = -1
        
        print str(data.shape[0]) + " : " + str(data.shape[1])
        
        for i in range(self.width):
            for j in range(self.height):
                if data[i,j] == 0:
                    labels = self.getNeighborLabels(i, j)
                    if len(labels) == 0:
                        self.labelData[i,j] = uf.createLabel()
                    else:
                        min_label = np.min(labels)
                        self.labelData[i,j] = min_label
                        for l in labels:
                            for nl in labels:
                                if l != nl:
                                    uf.union(l, nl)

        
        for i in range(self.width):
            for j in range(self.height):  
                if self.labelData[i, j] >= 0:  
                    self.labelData[i,j] = uf.find(self.labelData[i,j])
        
        self.labels = []
        for i in range(self.width):
            for j in range(self.height):  
                if self.labelData[i, j] >= 0: 
                    if not (self.labelData[i,j] in self.labels):
                        self.labels.append(self.labelData[i,j])    
                        
        self.components = []
        for l in self.labels:
            self.components.append([])    
        for i in range(self.width):
            for j in range(self.height):  
                if self.labelData[i, j] >= 0: 
                    self.labelData[i, j] = self.labels.index(self.labelData[i,j])
        for idx in range(len(self.labels)):
            self.labels[idx] = idx
            
        for i in range(self.width):
            for j in range(self.height):  
                if self.labelData[i, j] >= 0:
                    self.components[self.labelData[i, j]].append([i,j]) 
                    
    def getNeighborLabels(self, i, j):
        nlabels = []
        for op in neighbor_operators:
            nx = i + op[0]
            ny = j + op[1]
            if nx >= 0 and nx < self.width and ny >= 0 and ny < self.height:
                if self.data[nx, ny] == 0 and self.labelData[nx, ny] >= 0:
                    label = self.labelData[nx, ny]
                    if not (label in nlabels):
                        nlabels.append(label)
        return nlabels
    
    def getComponentNum(self):        
        return len(self.labels)
    
    def getComponent(self, idx):
        pixels = []
        for i in range(self.width):
            for j in range(self.height):
                if self.labelData[i,j] == idx:
                    pixels.append([i, j])
        return pixels
   
    def getComponentLabelData(self, idx):  
        labelData = np.zeros((self.width, self.height), np.int)
        for i in range(self.width):
            for j in range(self.height):
                if self.labelData[i, j] == idx:
                    labelData[i,j] = 1
        return labelData
            
