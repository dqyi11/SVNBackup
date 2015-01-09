'''
Created on Jan 3, 2015

@author: daqing_yi
'''

from kdtree import *
import numpy as np
from scipy.misc import imread
from SubRRTstar import *

class MORRTstar(object):

    def __init__(self, sampling_range, segment_length, objective_num):
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
        
        self.referenceTrees = []
        self.subTrees = []
        
    def init(self, start, goal, costFuncs, weights, subproblem_num):
        self.start = start
        self.goal = goal
        self.subproblemNum = subproblem_num
        
        self.costFuncs = costFuncs
        self.weights = weights
        
        rnodes = []
        for k in range(self.objectiveNum):
            reftree = SubRRTstar(self, [self.sampling_width, self.sampling_height], self.segmentLength, self.objectiveNum, k)
            reftree.root = RRTNode(start)
            reftree.nodes.append(reftree.root)
            reftree.root.cost = reftree.calcCost(reftree.root, None)
            self.referenceTrees.append(reftree)
            rnodes.append(reftree.root)
            
        for k in range(self.subproblemNum):
            subtree = SubRRTstar(self, [self.sampling_width, self.sampling_height], self.segmentLength, self.objectiveNum. self.objectiveNum+k)
            subtree.root = RRTNode(start)
            subtree.nodes.append(subtree.root)
            subtree.root.cost = subtree.calcCost(subtree.root, None)
            self.subTrees.append(subtree)
            rnodes.append(subtree.root)
        
        self.kdtree_root = createKDTree([start], self.dimension, ref_list=[rnodes])

        
    def loadMap(self, mapfile):
        self.mapfile = mapfile
        self.bitmap = np.array(imread(self.mapfile, True))
        self.sampling_width = self.bitmap.shape[1]
        self.sampling_height = self.bitmap.shape[0]
        
    
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
            nearest_pos, nearest_nodes = self.findNearestNeighbor(rndPos)
            
            new_pos = self.steer(rndPos, nearest_pos)
            
            if True == self.isObstacleFree(nearest_pos, new_pos):
                
                new_node_list = []
                
                # update reference trees (Reference trees are independent)
                for k in range(self.objectiveNum):
                    new_node = self.referenceTrees[k].addNewPos(nearest_nodes[k], new_pos)
                    new_node_list.append(new_node)
                    
                
                # add new pos to all the sub trees
                for k in range(self.subproblemNum):
                    new_node = self.subTrees[k].addNewPos(nearest_nodes[k+self.objectiveNum], new_pos)
                    new_node_list.append(new_node)
                
                # update rewired vertices of each sub trees
                
                self.kdtree_root.add(new_pos, new_node_list)
                
            
    def findNearVertices(self, pos, num):
        pos_list = []
        node_list = []
        results = self.kdtree_root.search_knn(pos, num)
        for res, dist in results:
            if res.data[0]==pos[0] and res.data[1]==pos[1]:
                continue
            node_list.append(res.ref)
            pos_list.append(res.data)
        return pos_list, node_list       
    
    def findNearestNeighbor(self, pos):
        results = self.kdtree_root.search_nn(pos)
        #print results[0], results[0].ref
        return results[0].data, results[0].ref
    
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
                          
    def findPaths(self):
        paths = []
        
        nearest_to_goal = self.kdtree_root.search_nn(self.goal)
        
        for k in range(self.objectiveNum):
            path = self.referenceTrees[k].findPath(nearest_to_goal)
            path.append([self.goal[0], self.goal[1]])
            paths.append(path)      
            
        for k in range(self.subproblemNum):
            path = self.subTrees[k].findPath(nearest_to_goal)
            path.append([self.goal[0], self.goal[1]])
            paths.append(path)
        
        return paths


                        
                
        
        
    
