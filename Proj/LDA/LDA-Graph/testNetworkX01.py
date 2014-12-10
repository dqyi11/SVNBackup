'''
Created on Dec 7, 2014

@author: daqing_yi
'''

import networkx as nx

if __name__ == '__main__':
    
    G = nx.Graph()
    G.add_node('3')
    G.add_node('1')
    G.add_node('2')
    G.add_node('3')
    G.add_node('1')
    G.add_node('1')
    G.add_node('3')
    G.add_node('2')
    G.add_node('1')
    G.add_node('1')
    G.add_node('2')
    G.add_node('3')
    G.add_node('1')
    G.add_node('2')
    G.add_node('3')
    
    print G.number_of_nodes()
    