'''
Created on Nov 20, 2014

@author: daqing_yi
'''

import networkx as nx

def visualizeNetwork(matrix):
    
    G = nx.Graph()
    w = matrix.shape[0]
    h = matrix.shape[1]
    
    for i in range(w):
        for j in range(h):
            t = int(matrix[i,j])
            #for tt in range(t):
            if t > 0:
                G.add_edge(i,j)
            
    nx.write_graphml(G, 'so.graphml')
    
    