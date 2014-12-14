'''
Created on Dec 13, 2014

@author: daqing_yi
'''

from KDResult import *


class KDNode(object):
    
    def __init__(self, pos):
        self.pos = pos
        self.left = None
        self.right = None
        self.data = None
  

class KDTree(object):

    def __init__(self, dimension):
        self.dimension = dimension
        self.root = None
        
    def insert(self, pos, data):

        self.insert_to_node(self.root, pos, data, 0)
        
        if self.rect == 0:
            self.rect = self.hyperrect_create(pos, pos)
        else:
            self.rect = self.hyperrect_extend(pos)
            
    def insert_to_node(self, node, pos, data, dir):
    
        if node==None:
            new_node = KDNode(pos)
            new_node.data = data
            new_node.dir = dir
            return
            
        new_dir = (node.dir + 1) % self.dimension
        if pos[node.dir] < node.pos[node.dir]:
            return self.insert_to_node(node.left, pos, data, new_dir)
        return self.insert_to_node(node.right, pos, data, new_dir)
            
    def findNearest(self, pos):
        # Find one of the nearest nodes from the specified point.
        
        
        
    def findNearestRange(self, pos, range):
        # Find any nearest nodes from the specified point within a range.
        
    
        
        
    
        
    

        