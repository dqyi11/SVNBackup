'''
Created on Jan 14, 2015

@author: daqing_yi
'''

import networkx as nx

class TreeAnalyzer(object):


    def __init__(self, tree):
        self.tree = tree
        self.G = nx.Graph()
        for n in tree.nodes:
            for c in n.children:
                from_node_str = "["+str(int(n.pos[0]))+","+str(int(n.pos[1]))+"]"
                to_node_str = "["+str(int(c.pos[0]))+","+str(int(c.pos[1]))+"]"
                self.G.add_edge(from_node_str, to_node_str)
                
    def hasCircle(self):
        circleNum = len(nx.cycle_basis(self.G))
        if circleNum > 0:
            return True
        return False
                
        
        

        