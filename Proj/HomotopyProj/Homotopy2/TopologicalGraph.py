'''
Created on Nov 17, 2014

@author: daqing_yi
'''

from graphviz import Graph

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
        
    def addNode(self, name):
        n = TopologicalNode(name)
        self.nodes.append(n)
        return n
        
    def addEdge(self, node_a, node_b, name):
        e = TopologicalEdge(node_a, node_b, name)
        self.edges.append(e)
        return e
    
    def findNode(self, name):
        for n in self.nodes:
            if n.name == name:
                return n
        return None
        
    def visualize(self, filename):
        
        g = Graph(format='png')
        for n in self.nodes:
            g.node(n.name)
        
        for e in self.edges:
            if e!= None:
                g.edge(e.node_a.name, e.node_b.name, label=e.name)
        
        g.render(filename+'.png', view=True)
        
        
        

        