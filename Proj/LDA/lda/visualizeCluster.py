'''
Created on Dec 11, 2014

@author: daqing_yi
'''

import numpy as np
import matplotlib.pyplot as plt

def visualizeCluster(clustered, type_ref, cluster_num, data_num, name):
    
    clusters = {}
    for k in range(cluster_num):
        clusters[k] = []
        
    for i in range(data_num):
        idx = clustered[0, i]
        clusters[idx].append(i)
    
    for k in range(cluster_num):
        hist_cnt = np.zeros(cluster_num)
        for s in clusters[k]:
            hist_cnt[type_ref[0, s]] += 1
            
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.bar(np.arange(cluster_num), hist_cnt, 0.4)
        plt.savefig(name+"-"+str(k)+".png")
            
       
            
        