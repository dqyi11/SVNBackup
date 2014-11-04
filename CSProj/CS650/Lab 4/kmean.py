'''
Created on Oct 30, 2014

@author: daqing_yi
'''

import numpy as np

class KMeanCluster(object):

    def __init__(self, data_dim, cluster_num, max_iter=1000, stop_err=0.00001):
        self.data_dim = data_dim
        self.cluster_num = cluster_num
        self.max_iter = max_iter
        self.stop_err = stop_err
        
    
    def cluster(self, data):
        self.data = data
        self.data_size = len(data)
        self.labels = np.zeros(self.data_size, np.int)
        
        self.means = []
        for i in range(self.cluster_num):
            self.means.append(np.zeros(self.data_dim))
        for i in range(self.cluster_num):
            select = int(i * self.data_size / self.cluster_num)
            for j in range(self.data_dim):
                self.means[i][j] = self.data[select][j]
                
        next_sums = []
        for j in range(self.cluster_num):
            next_sums.append(np.zeros(self.data_dim, np.double))
        counts = np.zeros(self.cluster_num, np.int)
        iter_cnt = 0
        current_cost = 0.0
        unchanged = 0
        
        loop = True
        while loop==True:
            
            last_cost = current_cost
            current_cost = 0.0
            # Classification
            for i in range(self.data_size):
                cost, l = self.getLabel(self.data[i])
                current_cost += cost
                counts[l] += 1

                for d in range(self.data_dim):
                    next_sums[l][d] += self.data[i][d]
                    
            current_cost = current_cost / self.data_size
            
            # Re-estimation
            for j in range(self.cluster_num):
                if counts[j] > 0:
                    for d in range(self.data_dim):
                        self.means[j][d] = next_sums[j][d] / counts[j]
            
            # Terminal condition
            iter_cnt += 1
            if np.abs(last_cost - current_cost) < self.stop_err * last_cost:
                unchanged += 1
            if iter_cnt > self.max_iter or unchanged >= 3:
                loop = False
        
    
    def getMean(self, idx):
        return self.means[idx]
    
    def getLabel(self, p):
        dist = -1
        label = None
        for i in range(self.cluster_num):
            temp = self.getDistance(p, self.means[i])
            if temp < dist or dist == -1:
                dist = temp
                label = i
        return dist, label

    
    def getDistance(self, p, q):
        
        dist = 0.0
        for j in range(self.data_dim):
            dist += (p[j] - q[j])**2
        return np.sqrt(dist)
    

        