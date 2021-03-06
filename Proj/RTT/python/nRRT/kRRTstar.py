'''
Created on Jan 3, 2015

@author: daqing_yi
'''

from kRRG import *

class kRRTstar(kRRG):

    def __init__(self, sampling_range, segmentLength):
        super(kRRTstar, self).__init__(sampling_range, segmentLength)
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
                
                min_new_node_cost = nearest_node.cost + self.costFunc(nearest_node, new_node)
                self.nodes.append(new_node)
                
                min_node = nearest_node
                
                near_node_list = self.findNearVertices(new_node.pos, self.nearNodeNum)
                
                for near_node in near_node_list:
                    if True == self.isObstacleFree(near_node.pos, new_node.pos):
                        c = near_node.cost + self.costFunc(near_node, new_node)
                        if c < min_new_node_cost:
                            min_node = near_node
                            min_new_node_cost = c
                            
                self.addEdge(min_node, new_node)
                new_node.cost = min_new_node_cost
                
                for near_node in near_node_list:
                    if near_node == min_node:
                        continue
                    
                    if True == self.isObstacleFree(new_node.pos, near_node.pos):
                        
                        delta_cost = near_node.cost - (new_node.cost + self.costFunc(new_node, near_node))
                        if delta_cost > 0:
                            parent_node = self.getParent(near_node)
                            self.removeEdge(parent_node, near_node)
                            self.addEdge(new_node, near_node)
                            self.updateCostToChildren(near_node, delta_cost)
                
                self.new_pos = [int(new_pos[0]), int(new_pos[1])]
                self.connected_pos = [int(nearest_node.pos[0]), int(nearest_node.pos[1])] 
                            
    def updateCostToChildren(self, node, delta_cost):
        node.cost = node.cost - delta_cost
        for cn in node.children:
            self.updateCostToChildren(cn, delta_cost)  
                            
    def getCostToNode(self, node):
        return node.cost
    
    def getParent(self, node):
        return node.parent
                 
                        
    def removeEdge(self, node_p, node_c):
        ret = super(kRRTstar, self).removeEdge(node_p, node_c)
        if ret == False:
            return False
        node_c.parent = None
        return True
    
    def addEdge(self, node_p, node_c):
        ret = super(kRRTstar, self).addEdge(node_p, node_c)
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
        path.append([goal_pos[0], goal_pos[1]])
        
        return path  

                        
                
        
        
    
