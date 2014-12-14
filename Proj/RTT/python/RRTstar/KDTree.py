'''
Created on Dec 13, 2014

@author: daqing_yi
'''

class KDNode(object):
    
    def __init__(self, pos):
        self.pos = pos
        self.left = None
        self.right = None
        

class KDTree(object):

    def __init__(self, dimension):
        self.dimension = dimension
        self.root = None
        
    def insert(self, pos, data):
        
        if self.root==None:
            new_node = KDNode(pos)
            self.root = new_node
            
    def findNearest(self, pos):
        # Find one of the nearest nodes from the specified point.
        
    def findNearestRange(self, pos, range):
        # Find any nearest nodes from the specified point within a range.
        
    
        
        
    
        
    

        