'''
Created on Jan 3, 2015

@author: daqing_yi
'''

from kdtree import *
import numpy as np
from scipy.misc import imread
from HomotopyMgr import *

class RRTNode(object):
    
    def __init__(self, pos):
        self.pos = pos
        self.parent = None
        self.children = []
        self.cost = 0.0
        
        self.strBit = None
        self.homoPath = []
        
    def __eq__(self, other):
        if other == None:
            return False
        if self.pos[0] == other.pos[0] and self.pos[1] == other.pos[1]:
            return True
        return False  

class BiRRTstar(object):

    def __init__(self, sampling_range, segmentLength):
        self.sampling_width = sampling_range[0]
        self.sampling_height = sampling_range[1]
        self.segmentLength = segmentLength
        self.dimension = 2
        self.st_nodes = []
        self.st_kdtree_root = None
        self.gt_nodes = []
        self.gt_kdtree_root = None
        self.bitmap = 255 * np.ones((self.sampling_height,self.sampling_width),np.int8)
        self.obsCheckResolution = 1
        self.mapfile = None
        
        self.st_new_node = None
        self.st_connected_node = None
        self.gt_new_node = None
        self.gt_connected_node = None
        
        self.nearNodeNum = 6
        self.gamma = 1.0
        self.radius = 600  #self.segmentLength
        self.costFunc = None
        
        self.dividingRefs = []
        
    def init(self, start, goal, costFunc, homotopyMgr):
        self.start = start
        self.goal = goal
        self.st_root = RRTNode(start)
        self.st_nodes.append(self.st_root)
        self.st_kdtree_root = createKDTree([start], self.dimension, ref_list=[self.st_root])
        
        self.gt_root = RRTNode(goal)
        self.gt_nodes.append(self.gt_root)
        self.gt_kdtree_root = createKDTree([start], self.dimension, ref_list=[self.gt_root])
        
        self.costFunc = costFunc
        self.st_root.cost = self.costFunc(self.st_root.pos, None)
        self.gt_root.cost = self.costFunc(self.gt_root.pos, None)
        self.homotopyMgr = homotopyMgr
        
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
        new_pos[0] = pos_b[0] + delta[0]
        new_pos[1] = pos_b[1] + delta[1]
        return new_pos
    
    def sampling(self):
        rndPos = np.random.random(self.dimension)
        rndPos[0] = rndPos[0]*self.sampling_width
        rndPos[1] = rndPos[1]*self.sampling_height
        
        return rndPos

        
    def extend(self, kdtree_root, nodes):
        new_node = None
        while new_node == None:
            rndPos = self.sampling()
            nearest_node = self.findNearestNeighbor(kdtree_root, rndPos)
            
            new_pos = self.steer(rndPos, nearest_node.pos)
            if self.isInObstacle(new_pos):
                continue
            
            if True == self.isObstacleFree(nearest_node.pos, new_pos):
                homoRet, new_homo_path, new_bit = self.isHomotopyFree(nearest_node, new_pos)
                #if homoRet == True:
                intcross = self.homotopyMgr.isCrossingDividingRefs(nearest_node.pos, new_pos)
                if intcross==False:
                    new_node = RRTNode(new_pos)
                    kdtree_root.add(new_pos, new_node)
                
                    min_new_node_cost = nearest_node.cost + self.costFunc(nearest_node.pos, new_node.pos)
                    nodes.append(new_node)
                
                    min_node = nearest_node
                    min_homo_path = new_homo_path
                    min_bit = new_bit
                
                    near_node_list = self.findNearVertices(kdtree_root, new_node.pos)
                
                    for near_node in near_node_list:
                        if True == self.isObstacleFree(near_node.pos, new_node.pos):
                            homoRet, new_homo_path, new_bit = self.isHomotopyFree(near_node, new_pos)
                            #if homoRet == True:
                            intcross = self.homotopyMgr.isCrossingDividingRefs(near_node.pos, new_pos)
                            if intcross == False:
                                c = near_node.cost + self.costFunc(near_node.pos, new_node.pos)
                                if c < min_new_node_cost:
                                    min_node = near_node
                                    min_new_node_cost = c
                                    min_homo_path = new_homo_path
                                    min_bit = new_bit
                            
                    self.addEdge(min_node, new_node)
                    new_node.cost = min_new_node_cost
                    new_node.homoPath = min_homo_path
                    new_node.strBit = min_bit
                    
                    for near_node in near_node_list:
                        if near_node == min_node:
                            continue
                        
                        if True == self.isObstacleFree(new_node.pos, near_node.pos):                 
                            homoRet, new_homo_path, new_bit = self.isHomotopyFree(new_node, near_node.pos)
                            #if homoRet == True:
                            intcross = self.homotopyMgr.isCrossingDividingRefs(new_node.pos, near_node.pos)
                            if intcross == False:
                                delta_cost = near_node.cost - (new_node.cost + self.costFunc(new_node.pos, near_node.pos))
                                if delta_cost > 0:
                                    parent_node = near_node.parent
                                    self.removeEdge(parent_node, near_node)
                                    self.addEdge(new_node, near_node)
                                    self.updateCostToChildren(near_node, delta_cost)
                                    near_node.homoPath = new_homo_path
                                    near_node.strBit = new_bit
                                    
            
    def findNearVertices(self, kdtree_root, pos):
        return self.findNearVerticesByRadius(kdtree_root, pos, self.radius)   
    
    def findNearVerticesByRadius(self, kdtree_root, pos, radius):
        node_list = []
        results = kdtree_root.search_nn_dist(pos, radius)

        for res in results:
            if res.data[0]==pos[0] and res.data[1]==pos[1]:
                continue
            node_list.append(res.ref)
        return node_list       
    
    def findNearestNeighbor(self, kdtree_root, pos):
        results = kdtree_root.search_nn(pos)
        #print results[0], results[0].ref
        return results[0].ref
    
    def isConnectableToGoal(self, pos):
        dist = np.sqrt((pos[0]-self.goal[0])**2+(pos[1]-self.goal[1])**2)
        if dist <= self.segmentLength:
            return True
        return False
    
    def isHomotopyFree(self, node, new_pos):
        
        newPath, newBit = self.homotopyMgr.extendPath(node.homoPath, node.pos, new_pos)
        ret =  self.homotopyMgr.inSameHomotopy(newPath)
        return ret, newPath, newBit
    
    
    
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
    
    def getSubpath(self, node, root):
        subpath = []
        
        node_list = []
        curr_node = node
        node_list.append(curr_node)
        while curr_node != root:
            curr_node = curr_node.parent
            node_list.append(curr_node)
            
        for n in reversed(node_list):
            subpath.append([int(n.pos[0]), int(n.pos[1])])
        
        return subpath
    
    def concatenatePaths(self, path1, path2):
        path = []
        stringInfo = []
        
        if len(path1) > 0:
            for idx1 in range(len(path1)-1):
                path.append([path1[idx1][0], path1[idx1+1][0]])
                if path1[idx1][1] != None:
                    stringInfo.append(path1[idx1][1]) 
            stringInfo.append(path1[len(path1)-1])          
            
        
        if len(path2) > 0:
            strBit = self.homotopyMgr.world_map.getCrossingSubsegment(path1[len(path1)-1][0], path2[0][0])
            path.append([path1[len(path1)-1][0], path2[0][0]])
            stringInfo.append(strBit)
            
            for idx2 in range(len(path2)-1, 0, -1):
                path.append([path2[idx2][0], path2[idx2-1][0]])
                if path2[idx2-1][1] != None:
                    stringInfo.append(path2[idx2-1][1]) 
            stringInfo.append(path2[0][1])
        
        return (path, stringInfo)
                          
    def findPaths(self):
        
        matchRadius = 15
        node_pairs = []
        paths = []
        # Find matching vertices from two trees
        for st_node in self.st_nodes:
            nearestNode = self.findNearestNeighbor(self.gt_kdtree_root, st_node.pos)
            dist = np.sqrt((nearestNode.pos[1]-st_node.pos[1])**2+(nearestNode.pos[0]-st_node.pos[0])**2)
            if dist < matchRadius:
                node_pairs.append((st_node, nearestNode))
        
        # create paths from subpaths
        for nodePair in node_pairs:
            
            subpathFromStart = self.getSubpath(nodePair[0], self.st_root)
            subpathFromGoal = self.getSubpath(nodePair[1], self.gt_root)
            
            path = self.concatenatePaths(subpathFromStart, subpathFromGoal)
            paths.append(path)
        
        return paths 


                        
                
        
        
    
