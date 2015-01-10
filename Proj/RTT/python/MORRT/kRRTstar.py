'''
Created on Jan 3, 2015

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

class kRRTstar(object):

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
        
        self.nearNodeNum = 6
        self.gamma = 1.0
        self.radius = self.segmentLength
        self.costFunc = None
        
    def init(self, start, goal, costFunc):
        self.start = start
        self.goal = goal
        self.root = RRTNode(start)
        self.nodes.append(self.root)
        self.kdtree_root = createKDTree([start], self.dimension, ref_list=[self.root])
        self.costFunc = costFunc
        self.root.cost = self.costFunc(self.root.pos, None)
        
    def loadMap(self, mapfile):
        self.mapfile = mapfile
        self.bitmap = np.array(imread(self.mapfile, True))
        self.sampling_width = self.bitmap.shape[1]
        self.sampling_height = self.bitmap.shape[0]
        
    def calcCost(self, node_a, node_b):
        cost = 0.0
        for k in range(self.objectiveNum):
            cost += self.costFunc[k](node_a, node_b) * self.weight[k]
        return cost
    
    def steer(self, pos_a, pos_b):
        
        # normalize along direction
        delta = np.zeros(self.dimension)
        delta[0] = pos_a[0] - pos_b[0]
        delta[1] = pos_a[1] - pos_b[1]
        delta_len = np.sqrt(delta[0]**2+delta[1]**2)
        scale = self.segmentLength/float(delta_len)
        delta = delta * scale
            
        new_pos = np.zeros(self.dimension)
        new_pos[0] = pos_b[0] + int(delta[0])
        new_pos[1] = pos_b[1] + int(delta[1])
        return new_pos
    
    def sampling(self):
        while True:
            rndPos = np.random.random(self.dimension)
            rndPos[0] = rndPos[0]*self.sampling_width
            rndPos[1] = rndPos[1]*self.sampling_height
            
            if False == self.isInObstacle(rndPos):
                return rndPos
        return None
        
    def extend(self):
        new_node = None
        while new_node == None:
            rndPos = self.sampling()
            nearest_node = self.findNearestNeighbor(rndPos)
            
            new_pos = self.steer(rndPos, nearest_node.pos)
            
            if True == self.isObstacleFree(nearest_node.pos, new_pos):
                new_node = RRTNode(new_pos)
                self.kdtree_root.add(new_pos, new_node)
                
                min_new_node_cost = nearest_node.cost + self.costFunc(nearest_node.pos, new_node.pos)
                self.nodes.append(new_node)
                
                min_node = nearest_node
                
                near_node_list = self.findNearVertices(new_node.pos, self.nearNodeNum)
                
                for near_node in near_node_list:
                    if True == self.isObstacleFree(near_node.pos, new_node.pos):
                        c = near_node.cost + self.costFunc(near_node.pos, new_node.pos)
                        if c < min_new_node_cost:
                            min_node = near_node
                            min_new_node_cost = c
                            
                self.addEdge(min_node, new_node)
                new_node.cost = min_new_node_cost
                
                for near_node in near_node_list:
                    if near_node == min_node:
                        continue
                    
                    if True == self.isObstacleFree(new_node.pos, near_node.pos):
                        
                        delta_cost = near_node.cost - (new_node.cost + self.costFunc(new_node.pos, near_node.pos))
                        if delta_cost > 0:
                            parent_node = near_node.parent
                            self.removeEdge(parent_node, near_node)
                            self.addEdge(new_node, near_node)
                            self.updateCostToChildren(near_node, delta_cost)
            
    def findNearVertices(self, pos, num):
        node_list = []
        results = self.kdtree_root.search_knn(pos, num)
        for res, dist in results:
            if res.data[0]==pos[0] and res.data[1]==pos[1]:
                continue
            node_list.append(res.ref)
        return node_list       
    
    def findNearestNeighbor(self, pos):
        results = self.kdtree_root.search_nn(pos)
        #print results[0], results[0].ref
        return results[0].ref
    
    def isConnectableToGoal(self, pos):
        dist = np.sqrt((pos[0]-self.goal[0])**2+(pos[1]-self.goal[1])**2)
        if dist <= self.segmentLength:
            return True
        return False
    
    def isObstacleFree(self, pos_a, pos_b):  
        obsFree = True
        
        if pos_a[0] == pos_b[0] and pos_a[1] == pos_b[1]:
            return obsFree
        
        x_dist = np.abs(pos_a[0] - pos_b[0])
        y_dist = np.abs(pos_a[1] - pos_b[1])
        
        if x_dist > y_dist:
            k = y_dist/x_dist
            if pos_a[0] < pos_b[0]:
                startX = pos_a[0]
                endX = pos_b[0]
                startY = pos_a[1]
            else:
                startX = pos_b[0]
                endX = pos_a[0]
                startY = pos_b[1]
            
            for coordX in np.arange(startX, endX+self.obsCheckResolution, self.obsCheckResolution):
                coordY = int(k*(coordX-startX) + startY)
                if coordY >= self.sampling_height or coordX >= self.sampling_width: break
                if self.bitmap[int(coordY),int(coordX)] < 255: 
                    obsFree = False
                if obsFree == False:
                    break
        else:
            k = x_dist/y_dist
            if pos_a[1] < pos_b[1]:
                startY = pos_a[1]
                endY = pos_b[1]
                startX = pos_a[0]
            else:
                startY = pos_b[1]
                endY = pos_a[1]
                startX = pos_b[0]
                
            for coordY in np.arange(startY, endY+self.obsCheckResolution, self.obsCheckResolution):
                coordX = int(k*(coordY-startY) + startX)
                if coordY >= self.sampling_height or coordX >= self.sampling_width: break
                if self.bitmap[int(coordY),int(coordX)] < 255: 
                    obsFree = False
                if obsFree == False:
                    break

        return obsFree

    def isInObstacle(self, pos):
        return False
                            
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
                          
    def findPath(self):
        path = []
        
        start_pos = np.zeros(2)
        goal_pos = np.zeros(2)
        start_pos[0], start_pos[1] = self.start[0], self.start[1]
        goal_pos[0], goal_pos[1] = self.goal[0], self.goal[1]
        
        nearest_to_goal = self.findNearestNeighbor(goal_pos)
        
        node_list = []
        curr_node = nearest_to_goal
        node_list.append(curr_node)
        while curr_node != self.root:
            curr_node = curr_node.parent
            node_list.append(curr_node)
            
        for n in reversed(node_list):
            path.append([int(n.pos[0]), int(n.pos[1])])
        path.append([goal_pos[0], goal_pos[1]])
        
        return path  


                        
                
        
        
    
