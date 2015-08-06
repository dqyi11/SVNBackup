'''
Created on Jan 3, 2015

@author: daqing_yi
'''

from kdtree import *
import numpy as np
from scipy.misc import imread
from SubkRRTstar import *

class MOkRRTstar(object):

    def __init__(self, sampling_range, segment_length, objective_num, subproblem_num):
        self.sampling_width = sampling_range[0]
        self.sampling_height = sampling_range[1]
        self.segmentLength = segment_length
        self.objectiveNum = objective_num
        self.subproblemNum = subproblem_num
        
        self.dimension = 2
        self.nodes = []
        self.kdtree_root = None
        self.bitmap = 255 * np.ones((self.sampling_height,self.sampling_width),np.int8)
        self.obsCheckResolution = 1
        self.mapfile = None
        
        self.new_pos = None
        self.connected_pos = None
        
        self.nearNodeNum = 6
        self.gamma = 1.0
        self.radius = self.segmentLength
        
        self.referenceTrees = []
        self.subTrees = []
        
    def init(self, start, goal, costFuncs, weights):
        self.start = start
        self.goal = goal
        
        self.costFuncs = costFuncs
        self.weights = weights
        
        rnodes = []
        for k in range(self.objectiveNum):
            reftree = RefkTree(self, [self.sampling_width, self.sampling_height], self.segmentLength, self.objectiveNum, k)
            weight = np.zeros(self.objectiveNum)
            weight[k] = 1.0
            reftree.init(start, goal, self.costFuncs, weight)
            reftree.root = RRTNode(start, self.objectiveNum)
            reftree.nodes.append(reftree.root)
            reftree.root.cost = reftree.calcCost(reftree.root, None)
            self.referenceTrees.append(reftree)
            rnodes.append(reftree.root)
            
        for k in range(self.subproblemNum):
            subtree = SubkTree(self, [self.sampling_width, self.sampling_height], self.segmentLength, self.objectiveNum, self.objectiveNum + k)
            subtree.init(start, goal, self.costFuncs, self.weights[k])
            subtree.root = RRTNode(start, self.objectiveNum)
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
        new_pos[0] = pos_b[0] + delta[0]
        new_pos[1] = pos_b[1] + delta[1]
        return new_pos
    
    def sampling(self):
        rndPos = np.random.random(self.dimension)
        rndPos[0] = rndPos[0]*self.sampling_width
        rndPos[1] = rndPos[1]*self.sampling_height
        
        return rndPos
        
    def extend(self):
        new_node = None
        while new_node == None:
            rndPos = self.sampling()
            nearest_pos, nearest_node_list = self.findNearestNeighbor(rndPos)
            
            new_pos = self.steer(rndPos, nearest_pos)
            if self.isInObstacle(new_pos):
                continue
            
            if True == self.isObstacleFree(nearest_pos, new_pos):
                
                near_poses_list, near_nodes_list = self.findNearVertices(new_pos, self.nearNodeNum)
                
                new_node_list = []
                
                # create new nodes of reference trees 
                for k in range(self.objectiveNum):
                    new_node = self.referenceTrees[k].createNewNode(new_pos)
                    new_node_list.append(new_node)
                    
                # create new nodes of sub trees 
                for k in range(self.subproblemNum):
                    new_node = self.subTrees[k].createNewNode(new_pos)
                    new_node_list.append(new_node)
                    
                self.kdtree_root.add(new_pos, new_node_list)
                
                # attach new node to reference trees
                # rewire near nodes of reference trees
                for k in range(self.objectiveNum):
                    self.referenceTrees[k].attachNewNode(new_node_list[k], nearest_node_list, near_nodes_list)
                    self.referenceTrees[k].rewireNearNodes(new_node_list[k], near_nodes_list)
                
                # attach new nodes to sub trees
                # rewire near nodes of sub trees
                for k in range(self.subproblemNum):
                    self.subTrees[k].attachNewNode(new_node_list[k+self.objectiveNum], nearest_node_list, near_nodes_list)
                    self.subTrees[k].rewireNearNodes(new_node_list[k+self.objectiveNum], near_nodes_list)

                self.new_pos = new_pos
                self.connected_pos = nearest_pos
                
    def getReferenceCost(self, pos):
        refCost = np.zeros(self.objectiveNum)
        result, dist = self.kdtree_root.search_nn(pos)
        
        if result.data[0] == pos[0] and result.data[1] == pos[1]:
            for k in range(self.objectiveNum):
                refCost[k] = result.ref[k].fitness
                
        return refCost
            
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
        results, dist = self.kdtree_root.search_nn(pos)
        #print results[0], results[0].ref
        return results.data, results.ref
    
    def isConnectableToGoal(self, pos):
        dist = np.sqrt((pos[0]-self.goal[0])**2+(pos[1]-self.goal[1])**2)
        if dist <= self.segmentLength:
            return True
        return False
    
    def isObstacleFree(self, pos_a, pos_b):  
        if pos_a[0] == pos_b[0] and pos_a[1] == pos_b[1]:
            return True
        
        x1 = int(pos_a[0])
        y1 = int(pos_a[1])
        x2 = int(pos_b[0])
        y2 = int(pos_b[1])
        
        dx = x2 - x1
        dy = y2 - y1
        is_steep = abs(dy) > abs(dx)
        if is_steep:
            x1, y1 = y1, x1
            x2, y2 = y2, x2
 
        swapped = False
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
            swapped = True
 
        dx = x2 - x1
        dy = y2 - y1
 
        error = int(dx / 2.0)
        ystep = 1 if y1 < y2 else -1
 
        y = y1
        points = []
        for x in range(x1, x2):
            coord = (y, x) if is_steep else (x, y)
            points.append(coord)
            if coord[1] >= self.bitmap.shape[0] or coord[0] >= self.bitmap.shape[1]:
                continue
            if self.bitmap[int(coord[1]),int(coord[0])] < 255:
                return False
            error -= abs(dy)
            if error < 0:
                y += ystep
                error += dx
                
        return True
    
    def isInObstacle(self, pos):
        return False
                            
    def updateCostToChildren(self, node, delta_cost):
        node.cost = node.cost - delta_cost
        for cn in node.children:
            self.updateCostToChildren(cn, delta_cost)
                          
    def findPaths(self):
        paths = []
        
        goal_pos = np.zeros(2)
        goal_pos[0], goal_pos[1] = self.goal[0], self.goal[1]
        nearest_pos_to_goal, nearest_to_goal = self.findNearestNeighbor(goal_pos)
        
        for k in range(self.objectiveNum):
            ref_tree = self.referenceTrees[k]
            path = ref_tree.findPath(nearest_to_goal[ref_tree.tree_idx])
            path.append([self.goal[0], self.goal[1]])
            paths.append(path)      
            
        for k in range(self.subproblemNum):
            sub_tree = self.subTrees[k]
            path = sub_tree.findPath(nearest_to_goal[sub_tree.tree_idx])
            path.append([self.goal[0], self.goal[1]])
            paths.append(path)
        
        return paths                     
                
        
        
    
