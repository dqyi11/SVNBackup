'''
Created on Dec 30, 2014

@author: daqing_yi
'''

from kdtree import *
import numpy as np
from scipy.misc import imread

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
        self.bitmap = 255 * np.ones((self.sampling_height,self.sampling_width),np.int8)
        self.obsCheckResolution = 1
        self.mapfile = None
        
        self.new_node = None
        self.connected_node = None
        
    def loadMap(self, mapfile):
        self.mapfile = mapfile
        self.bitmap = np.array(imread(self.mapfile, 'l'))
        self.sampling_width = self.bitmap.shape[1]
        self.sampling_height = self.bitmap.shape[0]
    
    def init(self, start, goal):
        self.start = start
        self.goal = goal
        self.root = RRTNode(start)
        self.nodes.append(self.root)
        self.kdtree_root = createKDTree([start], self.dimension, ref_list=[self.root])
        
    def extend(self):
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
    
            
            crossingObs = self.isCrossingObstacle(new_pos, nearest_node.pos)
            if False == crossingObs :
                #print new_pos
                new_node = RRTNode(new_pos)
                #new_node.cost = nearest_node.cost + self.segmentLength
                self.kdtree_root.add(new_pos, new_node)
                self.nodes.append(new_node)
                self.addEdge(nearest_node, new_node) 
                
                self.new_node = [int(new_pos[0]), int(new_pos[1])]
                self.connected_node = [int(nearest_node.pos[0]), int(nearest_node.pos[1])] 
        
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
        blocked = False
        
        x_dist = np.abs(pos_a[0] - pos_b[0])
        y_dist = np.abs(pos_a[1] - pos_b[1])
        stepLen = int(self.obsCheckResolution)
        
        if x_dist > y_dist:
            k = y_dist/x_dist
            if pos_a[0] < pos_b[0]:
                startX = int(pos_a[0])
                endX = int(pos_b[0])
                startY = pos_a[1]
            else:
                startX = int(pos_b[0])
                endX = int(pos_a[0])
                startY = pos_b[1]
            
            for coordX in range(startX, endX, stepLen):
                coordY = int(k*(coordX-startX) + startY)
                if coordY >= self.sampling_height or coordX >= self.sampling_width: break
                if self.bitmap[coordY,coordX] < 255: 
                    blocked = True
                if blocked: break
        else:
            k = x_dist/y_dist
            if pos_a[1] < pos_b[1]:
                startY = int(pos_a[1])
                endY = int(pos_b[1])
                startX = pos_a[0]
            else:
                startY = int(pos_b[1])
                endY = int(pos_a[1])
                startX = pos_b[0]
                
            for coordY in range(startY, endY, stepLen):
                coordX = int(k*(coordY-startY) + startX)
                if coordY >= self.sampling_height or coordX >= self.sampling_width: break
                if self.bitmap[coordY,coordX] < 255: 
                    blocked = True
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

        
        
