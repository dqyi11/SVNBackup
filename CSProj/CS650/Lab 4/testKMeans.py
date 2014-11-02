'''
Created on Oct 31, 2014

@author: daqing_yi
'''

import numpy as np
import matplotlib.pyplot as plt
from kmean import KMeanCluster

if __name__ == '__main__':
    
    data_dim = 2
    data_size = 200
    cluster_num = 6
    seeds = []
    for i in range(data_size):
        seeds.append(np.random.random(data_dim))
        
    markers = ['p', 'v', '^', '>', '<', 'D']    
        
    km = KMeanCluster(data_dim, cluster_num)
    km.cluster(seeds)
    
    labels = np.zeros(data_size, np.int)
    for i in range(data_size):
        s = seeds[i]
        dist, labels[i] = km.getLabel(s)
    label_color = []
    for i in range(cluster_num):
        label_color.append(tuple(np.random.random(3)))
        
    fig = plt.figure()
    ax = fig.add_subplot(111)
    for i in range(data_size):
        ax.plot(seeds[i][0], seeds[i][1],marker=markers[labels[i]],color=label_color[labels[i]])
    for j in range(cluster_num):
        ax.plot(km.means[j][0], km.means[j][1], 'o', color=label_color[j], markersize=10)
        ax.plot(km.means[j][0], km.means[j][1], 'x', color=(1, 0, 0))
    plt.show()
    