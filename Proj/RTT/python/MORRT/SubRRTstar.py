'''
Created on Jan 3, 2015

@author: daqing_yi
'''

from kdtree import *
import numpy as np


class RRTNode(object):
    
    def __init__(self, pos):
        self.pos = pos
        self.parent = None
        self.children = []
        self.cost = 0.0
        
    def __eq__(self, other):
        if other == None:
            return False
        if self.pos[0] == other.pos[0] and self.pos[1] == other.pos[1]:
            return True
        return False  

class SubRRTstar(object):

    def __init__(self, parent, sampling_range, segment_length, objective_num):
        self.parent = parent
        self.sampling_width = sampling_range[0]
        self.sampling_height = sampling_range[1]
        self.segmentLength = segment_length
        self.objectiveNum = objective_num
        
        self.dimension = 2
        self.nodes = []
        self.kdtree_root = None
        self.bitmap = 255 * np.ones((self.sampling_height,self.sampling_width),np.int8)
        self.obsCheckResolution = 1
        self.mapfile = None
        
        self.new_node = None
        self.connected_node = None
        
        self.nearNodeNum = 6
        self.gamma = 1.0
        self.radius = self.segmentLength
        self.costFuncs = None
        
    def init(self, start, goal, costFuncs, weights):
        self.start = start
        self.root = RRTNode(start)
        self.nodes.append(self.root)
        #self.kdtree_root = createKDTree([start], self.dimension, ref_list=[self.root])
        self.costFuncs = costFuncs
        self.weights = weights
        self.root.cost = self.calcCost(self.root, None)
        
        
    def addMewPos(self, new_pos):
        
        new_node = RRTNode(new_pos)
        
        return new_node
        
        
    def calcCost(self, node_a, node_b):
        cost = 0.0
        if node_a == None or node_b == None:
            return cost
        
        for k in range(self.objectiveNum):
            cost += self.costFuncs[k](node_a, node_b) * self.weight[k]
        return cost
                       
    def updateCostToChildren(self, node, delta_cost):
        node.cost = node.cost - delta_cost
        for cn in node.children:
            self.updateCostToChildren(cn, delta_cost)  
                                                   
    def removeEdge(self, node_p, node_c):
        if node_p == None:
            return False
        for c_a in node_p.children:
            if c_a == node_c:
                node_p.children.remove(c_a)
                c_a.parent = None
                return True
        return False

    
    def addEdge(self, node_p, node_c):
        for c_a in node_p.children:
            if c_a == node_c:
                return False
        node_p.children.append(node_c)
        node_c.parent = node_p
        return True
                          
    def findPath(self, nearest_to_goal):
        path = []
               
        node_list = []
        curr_node = nearest_to_goal
        node_list.append(curr_node)
        while curr_node != self.root:
            curr_node = curr_node.parent
            node_list.append(curr_node)
            
        for n in reversed(node_list):
            path.append([int(n.pos[0]), int(n.pos[1])])
        
        return path  


                        
                
        
        
    
