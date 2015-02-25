'''
Created on Nov 17, 2014

@author: daqing_yi
'''

#from graphviz import Graph
import networkx as nx
import matplotlib.pyplot as plt

class TopologicalNode(object):
    
    def __init__(self, name):
        self.name = name
        
class TopologicalEdge(object):
    
    def __init__(self, node_a, node_b, name=""):
        self.node_a = node_a
        self.node_b = node_b
        self.name = name

class TopologicalGraph(object):

    def __init__(self):
        self.nodes = []
        self.edges = []
        self.G = nx.Graph()
        self.edge_labels = {}
        
    def addNode(self, name):
        n = TopologicalNode(name)
        self.nodes.append(n)
        self.G.add_node(name, text=name)
        return n
        
    def addEdge(self, node_a, node_b, name):
        e = TopologicalEdge(node_a, node_b, name)
        self.edges.append(e)
        self.G.add_edge(node_a.name, node_b.name, text=name)
        self.edge_labels[node_a.name, node_b.name] = name
        return e
    
    def findNode(self, name):
        for n in self.nodes:
            if n.name == name:
                return n
        return None
        
    def visualize(self, filename):
        '''
        g = Graph(format='png', engine='neato')
        for n in self.nodes:
            g.node(n.name)
        
        for e in self.edges:
            if e!= None:
                g.edge(e.node_a.name, e.node_b.name, label=e.name)
        
        g.render(filename+'.png', view=True)
        '''
        graph_pos = nx.graphviz_layout(self.G)
        #graph_pos = nx.spring_layout(self.G, scale=4)
        nx.draw_networkx_edge_labels(self.G, pos=graph_pos, edge_labels=self.edge_labels)
        nx.draw_networkx_nodes(self.G, pos=graph_pos, node_size=1500, node_color=(153./255,178./255,255./255,1.0), wdith=0.0)
        nx.draw_networkx_edges(self.G, pos=graph_pos, width=3, edge_color='orange')
        nx.draw_networkx_labels(self.G, pos=graph_pos)
        plt.show()
             
        

        