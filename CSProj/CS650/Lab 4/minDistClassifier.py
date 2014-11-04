'''
Created on Nov 4, 2014

@author: daqing_yi
'''

import numpy as np

class MinDistClassifier(object):

    def __init__(self, data_dim, cluster_num, means):
        self.data_dim = data_dim
        self.cluster_num = cluster_num
        self.means = means
        
    def calcG(self, x, idx):
        dist = 0.0
        mm = 0.0
        for i in range(self.data_dim):
            dist += x[i]*self.means[idx][i]
            mm += self.means[idx][i]**2
        return dist-0.5*mm
        
        
    def getLabel(self, x):
        g_vals = np.zeros(self.cluster_num)
        for j in range(self.cluster_num):
            g_vals[j] = self.calcG(x, j)
        return g_vals.argmax()

        
        