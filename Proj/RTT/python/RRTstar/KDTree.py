'''
Created on Dec 13, 2014

@author: daqing_yi
'''

from KDResult import *
import copy

def KD_nearest_i(node, pos, rect):
    dir = node.dir
    result = None
    result_dist_sq = 0.0
    # Decide whether to go left or right in the tree
    dummy = pos[dir] - node.pos[dir]
    if dummy <= 0.0:
        nearer_subtree = node.left
        farther_subtree = node.right
        nearer_hyperrect_coord = rect.maxVal[dir]
        farther_hyperrect_coord = rect.minVal[dir]
        #side = 0
    else:
        nearer_subtree = node.right
        farther_subtree = node.left
        nearer_hyperrect_coord = rect.minVal[dir]
        farther_hyperrect_coord = rect.maxVal[dir]
        #side = 1  
        
    if nearer_subtree != None:
        # Slice the hyperrect to get the hyperrect of the nearer subtree
        dummy = nearer_hyperrect_coord
        nearer_hyperrect_coord = node.pos[dir]
        # Recursed down into nearer subtree
        result, result_dist_sq = KD_nearest_i(nearer_subtree, pos, rect)
        # Undo the slice
        nearer_hyperrect_coord = dummy
        
    # Check the distance of the point at the current node, compare it with our best so far
    dist_sq = 0.0
    for i in range(rect.dimension):
        deltaVal = node.pos[i] - pos[i]
        dist_sq += deltaVal * deltaVal
    
    if dist_sq < result_dist_sq:
        result = Node
        result_dist_sq = dist_sq
        
    if farther_subtree != None:
        # Get the hyperrect of the farther subtree
        dummy = farther_hyperrect_coord
        farther_hyperrect_coord = node.pos[dir]
        # Check if we have to recurse down by calculating the closest
        # point of the hyperrect and see if it's closer than our
        # minimum distance in result_dist_sq.
        if rect.distSquare(pos) < result_dist_sq:
            # Recurse down into farther subtree
            result, result_dist_sq = KD_nearest_i(farther_subtree, pos, rect)
        
        farther_hyperrect_coord = dummy
        
    return result, result_dist_sq
    

def KD_find_nearest(node, pos, pos_range, list, ordered, dim):
    if node==None:
        return 0
    
    added_results = 0
    
    dist_sq = 0.0
    for i in range(dim):
        deltaVal = node.pos[i] - pos[i]
        dist_sq += deltaVal * deltaVal
        
    if dist_sq <= pos_range * pos_range:
        if ordered == False:
            val = -1.0
        else:
            val = dist_sq
        ResultListInsert(list, node, val)
        added_results = 1
        
    dx = pos [node.dir] - node.pos[node.dir]
    child_node = node.left
    if dx <= 0.0:
        child_node = node.left
    else:
        child_node = node.right
    ret = KD_find_nearest(child_node, pos, pos_range, list, ordered, dim)
    
    if ret >= 0 and np.abs(dx) < pos_range:
        added_results += ret 
        child_node = node.left
        if dx <= 0.0:
            child_node = node.right
        else:
            child_node = node.left
        ret = KD_find_nearest(child_node, pos, pos_range, list, ordered, dim)
    
    added_results += ret 
    
    return added_results
        

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
        self.rect = None
        
    def insert(self, pos, data):
        self.root = self._insert_to_node(self.root, pos, data, 0)
        
        if self.rect == None:
            self.rect = KDHyperRect(self.dimension, pos, pos)
        else:
            self.rect.extend(pos)
            
    def _insert_to_node(self, node, pos, data, dir):
        if node==None:
            new_node = KDNode(pos)
            new_node.data = data
            new_node.dir = dir
            return new_node
            
        new_dir = (node.dir + 1) % self.dimension
        if pos[node.dir] < node.pos[node.dir]:
            node.left = self.insert_to_node(node.left, pos, data, new_dir)
            #node = temp_node
            return node
        node.right =  self.insert_to_node(node.right, pos, data, new_dir)
        return node
            
    def findNearest(self, pos):
        # Find one of the nearest nodes from the specified point.
        rset = KDResult()
        rset.tree = self
        rect = copy.deepcopy(self.rect)
        
        # Our first guesstimate is the root node
        result = self.root
        dist_sq = 0.0
        for i in range(self.dimension):
            deltaVal = result.pos[i] - pos[i]
            dist_sq += deltaVal * deltaVal
            
        # Search for the nearest neighbor recursively
        result, dist_sq = KD_nearest_i(self.root, pos, rect)
        
        # Store the result
        if result != None:
            ResultListInsert(rset.resList, result, -1.0)
            rset.size = 1
            rset.rewind()
        
        return rset
        
        
    def findNearestRange(self, pos, range):
        # Find any nearest nodes from the specified point within a range.
        rset = KDResult()
        
        ret = KD_find_nearest(self.root, pos, range, rset.resList, False, self.dimension)
        
        rset.size = ret
        rset.rewind()
        return rset
    
        
        
    
        
    

        