'''
Created on Jan 3, 2015

@author: daqing_yi
'''

from RRG import *

class RRTstar(RRG):

    def __init__(self, sampling_range, segmentLength):
        super(RRTstar, self).__init__(sampling_range, segmentLength)
        self.costFunc = None
        
    def extend(self):
        new_node = None
        while new_node == None:
            rndPos = self.sampling()
            nearest_node = self.findNearestNeighbor(rndPos)
            
            new_pos = self.steer(rndPos, nearest_node.pos)
            
            if True == self.isObstacleFree(nearest_node.pos, new_pos):
                new_node = RRTNode(new_pos)
                self.kdtree_root.add(new_pos, new_node)
                
                new_node.cost = nearest_node.cost + self.costFunc(new_node, nearest_node)
                self.nodes.append(new_node)
                
                min_node = nearest_node
                
                near_node_list = self.findNearVertices(new_node.pos, self.nearNodeNum)
                
                for near_node in near_node_list:
                    if True == self.isObstacleFree(near_node.pos, new_node.pos):
                        c = self.getCostToNode(near_node) + self.costFunc(near_node, new_node)
                        if c < self.getCostToNode(new_node):
                            min_node = near_node
                            
                self.addEdge(min_node, new_node)
                
                for near_node in near_node_list:
                    if near_node == min_node:
                        continue
                    
                    if True == self.isObstacleFree(new_node.pos, near_node.pos):
                        
                        if self.getCostToNode(near_node) > self.getCostToNode(new_node) + self.costFunc(new_node, near_node):
                            parent_node = self.getParent(near_node)
                            self.removeEdge(parent_node, near_node)
                            self.addEdge(new_node, near_node)
                
                            
                            
    def getCostToNode(self, node):
        return node.cost
    
    def getParent(self, node):
        return node.parent
                 
                        
    def removeEdge(self, node_p, node_c):
        ret = super(RRTstar, self).removeEdge(node_p, node_c)
        if ret == False:
            return False
        node_c.parent = None
        return True
    
    def addEdge(self, node_p, node_c):
        ret = super(RRTstar, self).addEdge(node_p, node_c)
        if ret == False:
            return False
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
        path.append(goal_pos)
        
        return path  

                        
                
        
        
    
