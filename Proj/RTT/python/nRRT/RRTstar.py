'''
Created on Jan 3, 2015

@author: daqing_yi
'''

from RRG import *

class RRTstar(RRG):

    def __init__(self, sampling_range, segmentLength):
        super(RRTstar, self).__init__(sampling_range, segmentLength)
        
    def extend(self):
        new_node = None
        while new_node == None:
            rndPos = self.sampling()
            nearest_node = self.findNearestNeighbor(rndPos)
            
            new_pos = self.steer(rndPos, nearest_node.pos)
            
            if True == self.isObstacleFree(new_pos, nearest_node.pos):
                new_node = RRTNode(new_pos)
                self.kdtree_root.add(new_pos, new_node)
                
                new_node.cost = nearest_node.cost + self.segmentLength
                self.nodes.append(new_node)
                
                min_node = nearest_node
                
                near_node_list = self.findNearVertices(new_node.pos, self.nearNodeNum)
                for near_node in near_node_list:
                    if True == self.isObstacleFree(near_node.pos, new_node.pos):
                        
                        c = self.getCostToNode(near_node) + self.getCost(near_node, new_node)
                        if c < self.getCostToNode(new_node):
                            min_node = near_node
                self.addEdge(min_node, new_node)
                
                for near_node in near_node_list:
                    if near_node == min_node:
                        continue
                    if True == self.isObstacleFree(near_node.pos, new_node.pos):
                        if self.getCostToNode(near_node) > self.getCostToNode(new_node) + self.getCost(new_node, near_node):
                            parent_node = self.getParent(near_node)
                            self.removeEdge(parent_node, near_node)
                            self.addEdge(new_node, near_node)
                            
    def getCostToNode(self, node):
        return self.getCost(self.root, node)
        
        
    def getCost(self, node_a, node_b):
        dist = np.sqrt((node_a.pos[0]-node_b.pos[0])**2+(node_a.pos[1]-node_b.pos[1])**2)
        return dist            
                        
    def removeEdge(self, node_a, node_b):
        ret = super(RRTstar, self).removeEdge(node_a, node_b)
        if ret == False:
            return False
        node_b.parent = None
        return True
    
    def addEdge(self, node_a, node_b):
        ret = super(RRTstar, self).addEdge(node_a, node_b)
        if ret == False:
            return False
        node_b.parent = node_a
        return True
                            

                        
                
        
        
    
