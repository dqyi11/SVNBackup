'''
Created on 2013-6-13

@author: Walter
'''

class ExtNode(object):
    
    def __init__(self, name, id):
        self.name = name
        self.id = id
    
    
class ExtEdge(object):
    
    def __init__(self, fromNode, toNode):
        self.fromNode = fromNode
        self.toNode = toNode

class NodeGraph(object):

    def __init__(self):
        self.nodes = []
        self.edges = []
        self.nodesDict = {}
    
    def importFromEdgeGraph(self, edgeGraph):
        
        pass
        
        
        