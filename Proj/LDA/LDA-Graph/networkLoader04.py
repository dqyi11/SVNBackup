'''
Created on Dec 10, 2014

@author: daqing_yi
'''

import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
import sys

def get_node_idx(G, node_name):
    idx = 0
    for n in G.nodes():
        if n== node_name:
            return idx
        idx += 1                
    return None

if __name__ == '__main__':
    
    Authors = []
    Adjacencies = []
    with open('author_nodes.txt') as fp:
        cnt = 0
        for line in fp:
            if cnt%2==0:
                Authors.append(line)
            elif cnt%2==1:
                line = line.replace("[", "")
                line = line.replace("]", "")
                xs = line.split(",")
                Adjacencies.append(xs)

            cnt += 1
        
        print len(Authors)
        print len(Adjacencies)
        
        print Adjacencies[1]
        
    node_num = len(Authors)
    G = nx.Graph()

    for i in range(node_num):
        for aj in Adjacencies[i]:
            G.add_edge(Authors[i], aj)
        
    G_node_num = G.number_of_nodes()
    G_edge_num = G.number_of_edges()
    print "G Node num " + str(G_node_num)
    print "G Edge num " + str(G_edge_num)
    
    data_mat = np.zeros((G_node_num, G_node_num))
    for e in G.edges():
        idx1 = get_node_idx(G, e[0])
        idx2 = get_node_idx(G, e[1])
        data_mat[idx1, idx2] += 1
    
    pos = nx.spring_layout(G)
    nx.draw_networkx_nodes(G, pos, node_color='r')
    nx.draw_networkx_edges(G, pos, edge_color='b')
    
    nx.write_graphml(G, 'net_load04.graphml')
    
    out_data = {}
    out_data["NODES"] = G.nodes()
    out_data["DATA_MATRIX"] = data_mat
    
    import scipy.io
    scipy.io.savemat('net_load04.mat', out_data)