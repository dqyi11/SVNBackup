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
        if self.pos[0] == other.pos[0] and self.pos[1] == other.pos[1]:
            return True
        return False        

class RRT(object):
    
    def __init__(self, sampling_range, segmentLength):
        self.sampling_width = sampling_range[0]
        self.sampling_height = sampling_range[1]
        self.segmentLength = segmentLength
        self.dimension = 2
        self.nodes = []
        self.kdtree_root = None
        self.bitmap = np.zeros((self.sampling_width, self.sampling_height),np.int8)
    
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
            delta[0] = rndPos[0] - nearest_node.pos[0]
            delta[1] = rndPos[1] - nearest_node.pos[1]
            delta_len = np.sqrt(delta[0]**2+delta[1]**2)
            scale = self.segmentLength/float(delta_len)
            delta = delta * scale
            
            new_pos = np.zeros(self.dimension)
            new_pos[0] = nearest_node.pos[0] + int(delta[0])
            new_pos[1] = nearest_node.pos[1] + int(delta[1])
            
            if False == self.isCrossingObstacle(new_pos, nearest_node.pos):
                print new_pos
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
        step = np.zeros(self.dimension)
        step[0] = (pos_a[0] - pos_b[0])/float(self.segmentLength)
        step[1] = (pos_a[1] - pos_b[1])/float(self.segmentLength)
        
        #Set small steps to check for walls
        
        pointsNeeded = int(math.floor(np.max(np.abs(step))))
        if math.fabs(step[0])>math.fabs(step[1]):
            if step[0] >= 0: 
                step = [1, step[1]/math.fabs(step[0])]
            else: 
                step = [-1, step[1]/math.fabs(step[0])]
        else:
            if step[1] >= 0: 
                step = [step[0]/math.fabs(step[1]), 1]
            else: 
                step = [step[0]/math.fabs(step[1]), -1]
  
        blocked = False
        for i in range(pointsNeeded+1): #Creates points between graph and solitary point
            for j in range(int(self.segmentLength)):   #Check if there are walls between points
                coordX = round(pos_a[0]+step[0]*j)
                coordY = round(pos_b[1]+step[1]*j)
                if coordX == pos_a[0] and coordY == pos_a[1]: break
                if coordY >= self.bitmap.shape[0] or coordX >= self.bitmap.shape[1]: break
                if self.bitmap[coordY,coordX] < 255: blocked = True
                if blocked: break
            if blocked: break
    
        return blocked

    
    def isInObstacle(self, pos):
        return False
    
    def generateRandomPos(self):
        while True:
            rndPos = np.random.random(self.dimension)
            rndPos[0] = rndPos[0]*self.sampling_width
            rndPos[1] = rndPos[1]*self.sampling_height
            
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

        
        
