'''
Created on Dec 30, 2014

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
        for d in range(len(self.pos)):
            if self.pos[d] != other.pos[d]:
                return False
        return True        

class RRT(object):
    
    def __init__(self, sampling_range, segmentLength):
        self.sampling_range = sampling_range
        self.dimension = len(self.sampling_range)
        self.sampling_width = np.zeros(self.dimension)
        for d in range(self.dimension):
            self.sampling_width[d] = self.sampling_range[d][1]-self.sampling_range[d][0]
        self.segmentLength = segmentLength
        self.nodes = []
        self.kdtree_root = None
        
    
    def init(self, start, goal):
        self.start = start
        self.goal = goal
        self.root = RRTNode(start)
        self.nodes.append(self.root)
        self.kdtree_root = createKDTree([start], self.dimension, ref_list=[self.root])
        
    def expand(self):
        new_node = None
        while new_node == None:
            rndPos = self.generateRandomPos()
            nearest_node = self.findClosetNode(rndPos)
            
            # normalize along direction
            delta = np.zeros(self.dimension)
            delta_len = 0.0
            for d in range(self.dimension):
                delta[d] = rndPos[d] - nearest_node.pos[d]
                delta_len += delta[d]**2
            delta_len = np.sqrt(delta_len)
            scale = self.segmentLength/float(delta_len)
            delta = delta * scale
            
            new_pos = np.zeros(self.dimension)
            for d in range(self.dimension):
                new_pos[d] = nearest_node.pos[d] + int(delta[d])
            
            if False == self.isCrossingObstacle(new_pos, nearest_node.pos):
                new_node = RRTNode(new_pos)
                #new_node.cost = nearest_node.cost + self.segmentLength
                self.kdtree_root.add(new_pos, new_node)
                self.nodes.append(new_node)
                self.addEdge(nearest_node, new_node)  
        
    def findClosetNode(self, pos):
        '''
        node_num = len(self.nodes)
        dist = np.zeros(node_num, np.float)
        for i in range(node_num):
            dist[i] = np.sqrt((pos[0]-self.nodes[i].pos[0])**2+(pos[1]-self.nodes[i].pos[1])**2)
        min_dist_idx = dist.argmin()
        return self.nodes[min_dist_idx]
        '''
        results = self.kdtree_root.search_nn(pos)
        #print results[0], results[0].ref
        return results[0].ref
    
    def isConnectableToGoal(self, pos):
        dist = np.sqrt((pos[0]-self.goal[0])**2+(pos[1]-self.goal[1])**2)
        if dist <= self.segmentLength:
            return True
        return False
    
    def isCrossingObstacle(self, pos_a, pos_b):
        return False
    
    def isInObstacle(self, pos):
        return False
    
    def generateRandomPos(self):
        while True:
            rndPos = np.random.random(self.dimension)
            for d in range(self.dimension):
                rndPos[d] = rndPos[d]*self.sampling_width[d] + self.sampling_range[d][0]
            
            if False == self.isInObstacle(rndPos):
                return rndPos
        return None
    
    def removeEdge(self, node_a, node_b):
        for c_a in node_a.children:
            if c_a == node_b:
                node_a.children.remove(c_a)
                return True
        return False
    
    def addEdge(self, node_a, node_b):
        for c_a in node_a.children:
            if c_a == node_b:
                return False
        node_a.children.append(node_b)
        return True

        
        
