'''
Created on Dec 7, 2014

@author: daqing_yi
'''

import numpy as np
import networkx as nx
import matplotlib.pyplot as plt

if __name__ == '__main__':
    
    def get_node_idx(G, node_name):
        idx = 0
        for n in G.nodes():
            if n== node_name:
                return idx
            idx += 1                
        return None

    citeseer_cites = np.genfromtxt('citeseer.cites', delimiter='\t', dtype=str)
    print citeseer_cites.shape
    
    G = nx.Graph()
    G1 = nx.Graph()
    G2 = nx.Graph()

    for cite in citeseer_cites:
        G.add_edge(cite[0], cite[1])
        G1.add_node(cite[0])
        G2.add_node(cite[1])
        
    G_node_num = G.number_of_nodes()
    G_edge_num = G.number_of_edges()
    print "G Node num " + str(G_node_num)
    print "G Edge num " + str(G_edge_num)
    
    G1_node_num = G1.number_of_nodes()
    G2_node_num = G2.number_of_nodes()
    print "G1 Node num " + str(G1_node_num)
    print "G2 Node num " + str(G2_node_num)
    
    print str(G1_node_num+G2_node_num)
        
    #pos = nx.spring_layout(G)
    
    data_mat = np.zeros((G_node_num, G_node_num))
    for e in G.edges():
        idx1 = get_node_idx(G, e[0])
        idx2 = get_node_idx(G, e[1])
        data_mat[idx1, idx2] += 1
    
    #nx.draw_networkx_nodes(G, pos, node_color='r')
    #nx.draw_networkx_edges(G, pos, edge_color='b')
    
    #nx.write_graphml(G, 'citeseer_cites.graphml')
    
    out_data = {}
    out_data["DATA_MATRIX"] = data_mat
    
    import scipy.io
    scipy.io.savemat('net_load02.mat', out_data)
    
    
    #plt.show()
    
    
    
    