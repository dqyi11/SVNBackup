'''
Created on Dec 11, 2014

@author: daqing_yi
'''

import numpy as np
import matplotlib.pyplot as plt

def visualizeCluster(clustered, type_ref, cluster_num, data_num, name):
    
    clusters = {}
    for k in range(cluster_num):
        clusters[0, k] = []
        
    for i in range(data_num):
        clusters[clustered[0, i]].append(i)
    
    for k in range(cluster_num):
        hist_cnt = np.zeros(cluster_num)
        for s in clusters[k]:
            hist_cnt[type_ref[0, s]] += 1
            
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.bar(hist_cnt)
        plt.savefig(name+"-"+str(k)+".png")
            
       
            
        