'''
Created on Oct 31, 2014

@author: daqing_yi
'''

import numpy as np
import matplotlib.pyplot as plt
from kmean import KMeanCluster
from minDistClassifier import *

if __name__ == '__main__':
    
    data_dim = 2
    data_size = 200
    cluster_num = 5
    seeds = []
    for i in range(data_size):
        seeds.append(np.random.random(data_dim))
        
    markers = ['p', 'v', '^', '>', '<']    
        
    km = KMeanCluster(data_dim, cluster_num)
    km.cluster(seeds)
    
    mdc = MinDistClassifier(data_dim, cluster_num, km.means)
    
    labels = np.zeros(data_size, np.int)
    for i in range(data_size):
        s = seeds[i]
        labels[i] = mdc.getLabel(s)
    label_color = [(1.0, 128.0/255, 0),
                   (0, 1.0, 128.0/255),
                   (0, 128.0/255, 1.0),
                   (1.0, 0, 128.0/255),
                   (128.0/255, 0, 1.0)]
        
    fig = plt.figure()
    ax = fig.add_subplot(111)
    for i in range(data_size):
        ax.plot(seeds[i][0], seeds[i][1],marker=markers[labels[i]],color=label_color[labels[i]])
    for j in range(cluster_num):
        ax.plot(km.means[j][0], km.means[j][1], 'o', color=label_color[j], markersize=10)
        ax.plot(km.means[j][0], km.means[j][1], 'x', color=(0, 0, 0))
    plt.show()
    