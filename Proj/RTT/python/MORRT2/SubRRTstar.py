'''
Created on Jan 3, 2015

@author: daqing_yi
'''

from kdtree import *
import numpy as np


class RRTNode(object):
    
    def __init__(self, pos, objective_num):
        self.pos = np.zeros(2)
        self.pos[0] = pos[0]
        self.pos[1] = pos[1]
        self.parent = None
        self.children = []
        self.objectiveNum = objective_num
        self.cost = np.zeros(self.objectiveNum)
        self.fitness = 0.0
        
    def __eq__(self, other):
        if other == None:
            return False
        if self.pos[0] == other.pos[0] and self.pos[1] == other.pos[1]:
            return True
        return False  


class ChildTree(object):

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
        
        self.nearNodeNum = 6
        self.gamma = 1.0
        self.radius = self.segmentLength
        
        
    def init(self, start, goal, costFuncs, weights):
        self.start = start
        self.root = RRTNode(start, self.objectiveNum)
        self.nodes.append(self.root)
        #self.kdtree_root = createKDTree([start], self.dimension, ref_list=[self.root])
        self.costFuncs = costFuncs
        self.weights = weights
        self.root.cost = self.calcCost(self.root.pos, None)
        
        
    def calcCost(self, node_a, node_b):
        cost = np.zeros(self.objectiveNum)
        if node_a == None or node_b == None:
            return cost
        
        for k in range(self.objectiveNum):
            cost[k] = self.costFuncs[k](node_a, node_b)
        return cost
    
    def calcFitness(self, cost, refCost):
        fitnessVals = np.zeros(self.objectiveNum)
        if refCost == None:
            for k in range(self.objectiveNum):
                fitnessVals[k] = self.weights[k] * cost[k]
        else:
            for k in range(self.objectiveNum):
                fitnessVals[k] = self.weights[k] * np.abs(cost[k] - refCost[k])
        return np.max(fitnessVals)   
    
    def createNewNode(self, new_pos):
        new_node = RRTNode(new_pos, self.objectiveNum)
        self.nodes.append(new_node)
        
        return new_node   
                                                   
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
            path.append([int(n.pos[0]), int(n.pos[1])])
        
        return path
    
    def findAllChildren(self, node):
        level = 0
        finished = False
        child_list = []
        current_level_nodes = []
        current_level_nodes.append(node)
        while finished == False:
            current_level_children = []
            for cln in current_level_nodes:
                current_level_children = list( set(current_level_children) | set(cln.children) )
            child_list = list( set(child_list) | set(current_level_children) )
            if len(current_level_children) == 0:
                finished = True
            else:
                current_level_nodes = current_level_children
                level += 1
            if level > 50:
                print "Probing level " + str(level)
        return child_list


class RefTree(ChildTree):
        
    def attachNewNode(self, new_node, nearest_node_list, near_nodes_list):
        nearest_node = nearest_node_list[self.tree_idx]
        min_new_node_cost = nearest_node.cost + self.calcCost(nearest_node.pos, new_node.pos)
        min_new_node_fitness = self.calcFitness(min_new_node_cost, None)
        min_node = nearest_node
        
        #near_pos_list, near_node_list = self.parent.findNearVertices(new_node.pos, self.nearNodeNum)
        
        for near_node_list in near_nodes_list:
            near_node = near_node_list[self.tree_idx]
            if True == self.parent.isObstacleFree(near_node.pos, new_node.pos):
                c = near_node.cost + self.calcCost(near_node.pos, new_node.pos)
                f = self.calcFitness(c, None)
                if f < min_new_node_fitness:
                    min_node = near_node
                    min_new_node_fitness = f
                    min_new_node_cost = c
                    
        self.addEdge(min_node, new_node)
        new_node.cost = min_new_node_cost
        new_node.fitness = min_new_node_fitness
        
    def rewireNearNodes(self, new_node, near_nodes_list):
        
        for near_node_list in near_nodes_list:
            near_node = near_node_list[self.tree_idx]
            if near_node == new_node or near_node == self.root or near_node == new_node.parent:
                continue
            
            if True == self.parent.isObstacleFree(new_node.pos, near_node.pos):
                
                temp_cost_from_new_node = new_node.cost + self.calcCost(new_node.pos, near_node.pos)
                temp_fitness_from_new_node = self.calcFitness(temp_cost_from_new_node, None)
                delta_cost = near_node.cost - temp_cost_from_new_node
                delta_fitness = self.calcFitness(near_node.cost, None) - temp_fitness_from_new_node
                
                if delta_fitness > 0:
                    parent_node = near_node.parent
                    self.removeEdge(parent_node, near_node)
                    self.addEdge(new_node, near_node)
                    near_node.cost = temp_cost_from_new_node
                    near_node.fitness = temp_fitness_from_new_node
                    self.updateCostToChildren(near_node, delta_cost)
                    
    def updateCostToChildren(self, node, delta_cost):
        '''
        node.cost = node.cost - delta_cost
        for cn in node.children:
            self.updateCostToChildren(cn, delta_cost)
        node.fitness = self.calcFitness(node.cost, None)
        '''
        child_list = self.findAllChildren(node)
        for c in child_list:
            c.cost = c.cost - delta_cost
            c.fitness = self.calcFitness(c.cost, None)
    
    
class SubTree(ChildTree):
    
    def attachNewNode(self, new_node, nearest_node_list, near_nodes_list):
        nearest_node = nearest_node_list[self.tree_idx]
        min_new_node_cost = nearest_node.cost + self.calcCost(nearest_node.pos, new_node.pos)
        min_new_node_fitness = self.calcFitness(min_new_node_cost, self.parent.getReferenceCost(new_node.pos))
        min_node = nearest_node
        
        #near_pos_list, near_node_list = self.parent.findNearVertices(new_node.pos, self.nearNodeNum)
        
        for near_node_list in near_nodes_list:
            near_node = near_node_list[self.tree_idx]
            if True == self.parent.isObstacleFree(near_node.pos, new_node.pos):
                c = near_node.cost + self.calcCost(near_node.pos, new_node.pos)
                f = self.calcFitness(c, self.parent.getReferenceCost(new_node.pos))
                if f < min_new_node_fitness:
                    min_node = near_node
                    min_new_node_fitness = f
                    min_new_node_cost = c
                    
        self.addEdge(min_node, new_node)
        new_node.cost = min_new_node_cost
        new_node.fitness = min_new_node_fitness
        
    def rewireNearNodes(self, new_node, near_nodes_list):
        
        for near_node_list in near_nodes_list:
            near_node = near_node_list[self.tree_idx]
            if near_node == new_node or near_node == self.root or near_node == new_node.parent:
                continue
            
            if True == self.parent.isObstacleFree(new_node.pos, near_node.pos):
                
                temp_cost_from_new_node = new_node.cost + self.calcCost(new_node.pos, near_node.pos)
                temp_fitness_from_new_node = self.calcFitness(temp_cost_from_new_node, self.parent.getReferenceCost(near_node.pos))
                delta_cost = near_node.cost - temp_cost_from_new_node
                delta_fitness = self.calcFitness(near_node.cost, self.parent.getReferenceCost(near_node.pos)) - temp_fitness_from_new_node
                
                if delta_fitness > 0:
                    parent_node = near_node.parent
                    self.removeEdge(parent_node, near_node)
                    self.addEdge(new_node, near_node)
                    near_node.cost = temp_cost_from_new_node
                    near_node.fitness = temp_fitness_from_new_node
                    self.updateCostToChildren(near_node, delta_cost)
                    
    def updateCostToChildren(self, node, delta_cost):
        '''
        node.cost = node.cost - delta_cost
        for cn in node.children:
            self.updateCostToChildren(cn, delta_cost) 
        refCost = self.parent.getReferenceCost(node.pos)
        node.fitness = self.calcFitness(node.cost, refCost) 
        '''
        child_list = self.findAllChildren(node)
        for c in child_list:
            c.cost = c.cost - delta_cost
            refCost = self.parent.getReferenceCost(c.pos)
            c.fitness = self.calcFitness(c.cost, refCost)
        
        
    
                
        
        
    
