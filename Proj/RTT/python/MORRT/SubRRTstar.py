'''
Created on Jan 3, 2015

@author: daqing_yi
'''

from kdtree import *
import numpy as np


class RRTNode(object):
    
    def __init__(self, pos):
        self.pos = np.zeros(2)
        self.pos[0] = pos[0]
        self.pos[1] = pos[1]
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

    def __init__(self, parent, sampling_range, segment_length, objective_num, tree_idx):
        self.parent = parent
        self.sampling_width = sampling_range[0]
        self.sampling_height = sampling_range[1]
        self.segmentLength = segment_length
        self.objectiveNum = objective_num
        
        self.tree_idx = tree_idx
        
        self.dimension = 2
        self.nodes = []
        self.kdtree_root = None
        self.bitmap = 255 * np.ones((self.sampling_height,self.sampling_width),np.int8)
        self.obsCheckResolution = 1
        self.mapfile = None
        
        self.new_node = None
        self.connected_node = None
        
        #self.nearNodeNum = 6
        self.gamma = 1.0
        self.radius = self.segmentLength
        
        
    def init(self, start, goal, costFuncs, weights):
        self.start = start
        self.root = RRTNode(start)
        self.nodes.append(self.root)
        self.costFuncs = costFuncs
        self.weights = weights
        self.root.cost = self.calcCost(self.root.pos, None)
        
        
    def createNewNode(self, new_pos):
        new_node = RRTNode(new_pos)
        self.nodes.append(new_node)
        return new_node
        
    def attachNewNode(self, new_node, nearest_node_list, near_nodes_list):
        nearest_node = nearest_node_list[self.tree_idx]
        min_new_node_cost = nearest_node.cost + self.calcCost(nearest_node.pos, new_node.pos)
        min_node = nearest_node
        
        #near_pos_list, near_node_list = self.parent.findNearVertices(new_node.pos, self.nearNodeNum)
        
        for near_node_list in near_nodes_list:
            near_node = near_node_list[self.tree_idx]
            if True == self.parent.isObstacleFree(near_node.pos, new_node.pos):
                c = near_node.cost + self.calcCost(near_node.pos, new_node.pos)
                if c < min_new_node_cost:
                    min_node = near_node
                    min_new_node_cost = c
                    
        ret = self.addEdge(min_node, new_node)
        if ret == False:
            print "Attach NEW NODE = add Edge " + str(min_node) + " " + str(new_node)
        new_node.cost = min_new_node_cost
        
    def rewireNearNodes(self, new_node, near_nodes_list):
        
        for near_node_list in near_nodes_list:
            near_node = near_node_list[self.tree_idx]
            if near_node == new_node or near_node == self.root:
                continue
            
            if True == self.parent.isObstacleFree(new_node.pos, near_node.pos):
                
                delta_cost = near_node.cost - (new_node.cost + self.calcCost(new_node.pos, near_node.pos))
                if delta_cost > 0:
                    parent_node = near_node.parent
                    ret = self.removeEdge(parent_node, near_node)
                    if ret == False:
                        print "REWIRE NODE = Remove Edge " + str(parent_node) + " " + str(near_node)
                    
                    ret = self.addEdge(new_node, near_node)
                    if ret == False:
                        print "REWIRE NODE = Add Edge " + str(new_node) + " " + str(near_node)
                    self.updateCostToChildren(near_node, delta_cost)
        
        
    def calcCost(self, node_a, node_b):
        cost = 0.0
        if node_a == None or node_b == None:
            return cost
        
        for k in range(self.objectiveNum):
            cost += self.costFuncs[k](node_a, node_b) * self.weights[k]
        return cost
                       
    def updateCostToChildren(self, node, delta_cost):
        node.cost = node.cost - delta_cost
        for cn in node.children:
            self.updateCostToChildren(cn, delta_cost)  
                                                   
    def removeEdge(self, node_p, node_c):
        if node_p == None:
            return False
        
        node_c.parent = None
        removed = False
        for c_a in node_p.children:
            if c_a == node_c:
                node_p.children.remove(c_a)
                c_a.parent = None
                removed = True
        return removed
    
    def hasEdge(self, node_p, node_c):
        if node_p == None or node_c == None:
            return False
        for c in node_p.children:
            if c == node_c:
                return True
        return False

    def addEdge(self, node_p, node_c):
        if node_p == node_c:
            return False
        if self.hasEdge(node_p, node_c):
            node_c.parent = node_p
            return True

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
            path.append([n.pos[0], n.pos[1]])
        
        return path  


                        
                
        
        
    
