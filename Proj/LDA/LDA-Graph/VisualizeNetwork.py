'''
Created on Nov 20, 2014

@author: daqing_yi
'''

import networkx as nx
import matplotlib.pyplot as plt
import numpy as np

def visualizeNetwork(matrix_e, matrix_c, colors):
    
    G = nx.Graph()
    h = matrix_e.shape[0]
    w = matrix_e.shape[1]
    
    print list(np.arange(h))
    
    print matrix_e.shape
    for i in range(h):
        for j in range(w):
            t = int(matrix_e[i,j])
            G.add_edge(i,t)
            c = int(matrix_c[i,j])
            G[i][t]['edge_color'] = colors[c]
            
    pos = nx.spring_layout(G)
    nx.draw_networkx_nodes(G, pos, nodelist=list(np.arange(h)), node_color='r')
    for i in range(h):
        for j in range(w):
            nx.draw_networkx_edges(G, pos, edgelist=[(i,t)], edge_color=colors[c])
            
    #nx.write_graphml(G, 'so.graphml')
    plt.show()
    
    