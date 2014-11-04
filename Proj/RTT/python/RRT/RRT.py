'''
Created on Oct 24, 2014

@author: daqing_yi
'''

import numpy as np

class RRTNode(object):
    
    def __init__(self, pos):
        self.pos = pos
        self.children = []
        self.cost = 0.0
        

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
            
            node = self.findClosetNode(rndPos)
            
            if self.isCrossingObstacle(rndPos, node.pos):
                new_node = RRTNode(rndPos)
                new_node.cost = node.cost + self.segmentLength
                
        
                            
        
        
    def findCLoseNode(self, pos):
        node_num = len(self.nodes)
        dist = np.zeros(node_num, np.float)
        for i in range(node_num):
            dist[i] = np.sqrt((pos[0]-self.nodes[i].pos[0])**2+(pos[1]-self.nodes[i].pos[1])**2)
        min_dist_idx = dist.amin()
        return self.notes[min_dist_idx]
    
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
    

        
        
        
    
    
    
    
            
    
        