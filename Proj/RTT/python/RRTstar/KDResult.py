'''
Created on Dec 13, 2014

@author: daqing_yi
'''
import numpy as np
from compiler.ast import Node

class ResultNode(object):
    
    def __init__(self):
        self.item = None
        self.dist_square = 0.0
        self.next = None
        
    
        
class KDResult(object):
    
    def __init__(self):
        self.tree = None
        self.resList = None
        self.resIter = None
        self.size = 0
        
    def getSize(self):
        return self.size
    
    def rewind(self):
        self.resIter = self.resList.next
        
    def isEnd(self):
        if self.resIter==None:
            return True
        return False
    
    def next(self):
        self.resIter = self.resList.next
        if self.resIter!= None:
            return True
        return False
    
    def item(self):
        pos = None
        data = None
        if self.resIter!=None:
            pos = self.resIter.item.pos
            data = self.resIter.item.data
        return pos, data
    
    def itemData(self):
        pos, data = self.item()
        return data
    
    def clear(self):
        self.resList.next = None
            
class KDHyperRect(object):
    
    def __init__(self, dimension, minVal, maxVal):
        self.dimension = dimension
        self.minVal = np.zeros(self.dimension)
        self.maxVal = np.zeros(self.dimension)
        for i in range(self.dimension):
            self.minVal[i] = minVal[i]
            self.maxVal[i] = maxVal[i]
        
    def extend(self, pos):
        for i in range(self.dimension):
            if pos[i] < self.minVal[i]:
                self.minVal[i] = pos[i]
            if pos[i] > self.maxVal[i]:
                self.maxVal[i] = pos[i]
                
    def distSquare(self, pos):
        result = 0.0
        for i in range(self.dimension):
            if pos[i] < self.minVal[i]:
                deltaVal = self.minVal[i]-pos[i]
                result += deltaVal * deltaVal
            elif pos[i] > self.maxVal[i]:
                deltaVal = self.maxVal[i]-pos[i]
                result += deltaVal* deltaVal
        return result
    
    
        
        
    
    
        
        
        
        
            
    