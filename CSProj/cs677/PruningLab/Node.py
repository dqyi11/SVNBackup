'''
Created on Jun 11, 2013

@author: joseph
'''

NodeType = {"Normal":0, "Query":1, "Evidence":2}

class Node:
    
    def __init__(self, name):
        # A list of all of the parent nodes of this node.
        self.parents = []
        # A list of all of the child nodes of this node.
        self.children = []
        self.name = name
        self.hasEvidenceDescendant = False
        self.type = NodeType["Normal"]
        self.isPrunable = False
    
    def __repr__(self):
        return self.name
        
        
