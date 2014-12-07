'''
Created on Dec 7, 2014

@author: daqing_yi
'''

import numpy as np
import networkx as nx
import matplotlib.pyplot as plt

if __name__ == '__main__':
    
    citeseer_content = np.loadtxt('citeseer.content', delimiter='\t', dtype=str)
    citeseer_cites = np.genfromtxt('citeseer.cites', delimiter='\t', dtype=str)
    
    print citeseer_content.shape
    print citeseer_cites.shape
    
    G = nx.Graph()

    for cite in citeseer_cites:
        G.add_edge(cite[0], cite[1])
        
    pos = nx.spring_layout(G)
    nx.draw_networkx_nodes(G, pos, node_color='r')
    nx.draw_networkx_edges(G, pos, edge_color='b')
    plt.show()
    
    
    
    