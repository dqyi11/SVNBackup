'''
Created on Oct 24, 2014

@author: daqing_yi
'''

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
        if self.pos[0]==other.pos[0] and self.pos[1]==other.pos[1]:
            return True
        return False        

class RRT(object):
    
    def __init__(self, dimension, segmentLength):
        self.dimension = dimension
        self.segmentLength = segmentLength
        self.nodes = []
        
    
    def init(self, start, goal):
        self.start = start
        self.goal = goal
        self.root = RRTNode(start)
        self.nodes.append(self.root)
        
    def expand(self):
        new_node = None
        while new_node == None:
            rndPos = self.generateRandomPos()
            nearest_node = self.findClosetNode(rndPos)
            
            # normalize along direction
            delta = [0.0,0.0]
            delta[0] = rndPos[0] - node.pos[0]
            delta[1] = rndPos[1] - node.pos[1]
            delta_len = np.sqrt(delta[0]**2+delta[1]**2)
            scale = self.segmentLength/float(delta_len)
            delta[0] = delta[0] * scale
            delta[1] = delta[1] * scale
            
            new_pos = [0, 0]
            new_pos[0] = nearest_node.pos[0] + int(delta[0])
            new_pos[1] = nearest_node.pos[1] + int(delta[1])
            
            if False == self.isCrossingObstacle(new_pos, nearest_node.pos):
                new_node = RRTNode(new_pos)
                new_node.cost = nearest_node.cost + self.segmentLength
                self.nodes.append(new_node)
                self.addEdge(nearest_node, new_node)  
        
    def findClosetNode(self, pos):
        node_num = len(self.nodes)
        dist = np.zeros(node_num, np.float)
        for i in range(node_num):
            dist[i] = np.sqrt((pos[0]-self.nodes[i].pos[0])**2+(pos[1]-self.nodes[i].pos[1])**2)
        min_dist_idx = dist.argmin()
        return self.nodes[min_dist_idx]
    
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
            rndPos = np.random.random(2)
            rndPos[0] = rndPos[0] * self.dimension[0]
            rndPos[1] = rndPos[1] * self.dimension[1]
            
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
    

        
        
        
    
    
    
    
            
    
        