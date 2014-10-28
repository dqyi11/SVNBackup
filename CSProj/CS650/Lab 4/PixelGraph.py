'''
Created on Oct 27, 2014

@author: daqing_yi
'''

#http://en.wikipedia.org/wiki/Connected-component_labeling

import numpy as np



class PixelGraph(object):

    def __init__(self, data):
        
        table = UnionFind()
        self.labelData = np.zeros(data.shape, np.int)
        self.width = data.shape[0]
        self.height = data.shape[1]
        self.data = data
        
        for i in range(1, self.width):
            for j in range(1, self.height):
                if data[i,j] == 0:
                    
                    if self.labelData[i, j-1] == 0:
                        self.labelData[i,j] = self.labelData[i, j-1]
                        if self.labelData[i-1, j] == 0:
                            table.union(self.labelData[i-1,j], self.labelData[i,j])
                    elif self.labelData[i-1, j] == 0:
                        self.labelData[i, j] = self.labelData[i-1, j]
                    else:
                        self.labelData[i,j] = table.set()
       
        for i in range(1, self.width):
            for j in range(1, self.height):        
                self.labelData[i,j] = table.find(self.labelData[i,j])         
         

        