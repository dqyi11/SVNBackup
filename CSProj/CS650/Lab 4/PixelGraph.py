'''
Created on Oct 27, 2014

@author: daqing_yi
'''

#http://en.wikipedia.org/wiki/Connected-component_labeling

import numpy as np
import cv2
import csv
import matplotlib.pyplot as plt
from UnionFindSet import *

neighbor_operators = [[-1, -1], [-1, 0], [-1, 1], [0, -1]]

class PixelGraph(object):

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
                #print data[i,j]
                if data[i,j] == 0:
                    labels = self.getNeighborLabels(i, j)
                    if len(labels) == 0:
                        self.labelData[i,j] = uf.createLabel()
                        #print self.labelData[i, j]
                    else:
                        #print labels
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
                    self.components[self.labelData[i, j]].append([i, j]) 
            
                    
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
    
    def visualize(self, name):
        
        color_img = np.zeros((self.width, self.height, 3), np.int)
        componenet_num = self.getComponentNum()
        r_vals = np.random.randint(0, 256, componenet_num)
        g_vals = np.random.randint(0, 256, componenet_num)
        b_vals = np.random.randint(0, 256, componenet_num)  
        
        for i in range(self.width):
            for j in range(self.height):
                if self.labelData[i,j] >= 0:
                    idx = self.labelData[i, j]
                    color_img[i, j, 0] = r_vals[idx]
                    color_img[i, j, 1] = g_vals[idx]
                    color_img[i, j, 2] = b_vals[idx]
                else:
                    color_img[i, j, 0] = 255
                    color_img[i, j, 1] = 255
                    color_img[i, j, 2] = 255
                            
        plt.imshow(color_img)
        plt.show()
        
        
    def visualizeComponent(self, name, idx):
        
        color_img = np.zeros((self.width, self.height, 3), np.int)
        r_val = np.random.randint(0, 256)
        g_val = np.random.randint(0, 256)
        b_val = np.random.randint(0, 256)  
        
        for c in self.components[idx]:
            color_img[c[0], c[1], 0] = r_val
            color_img[c[0], c[1], 1] = g_val
            color_img[c[0], c[1], 2] = b_val
                            
        plt.imshow(color_img)
        plt.show()
        
    def dump(self, filename):
        with open(filename, 'wb') as f:
            writer = csv.writer(f)
            writer.writerows(self.labelData)
        
        
                     

        