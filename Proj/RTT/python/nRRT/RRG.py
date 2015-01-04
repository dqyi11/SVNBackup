'''
Created on Jan 3, 2015

@author: daqing_yi
'''

from RRT import *

class RRG(RRT):

    def __init__(self, sampling_range, segmentLength):
        super(RRG, self).__init__(sampling_range, segmentLength)
        self.nearNodeNum = 3
        
    def extend(self):
        new_node = None
        while new_node == None:
            rndPos = self.sampling()
            nearest_node = self.findNearestNeighbor(rndPos)
            
            new_pos = self.steer(rndPos, nearest_node.pos)
            
            if True == self.isObstacleFree(new_pos, nearest_node.pos):
                new_node = RRTNode(new_pos)
                #new_node.cost = nearest_node.cost + self.segmentLength
                
                self.kdtree_root.add(new_pos, new_node)
                self.nodes.append(new_node)
                self.addEdge(nearest_node, new_node)
                self.addEdge(new_node, nearest_node)
                
                near_node_list = self.findNearVertices(new_node.pos, self.nearNodeNum)
                for near_node in near_node_list:
                    if near_node != new_node:
                        if True == self.isObstacleFree(near_node.pos, new_node.pos):
                            self.addEdge(near_node, new_node)
                            self.addEdge(near_node, new_node)
                
    def findNearVertices(self, pos, num):
        node_list = []
        results = self.kdtree_root.search_knn(pos, num)
        for res in results:
            node_list.append(res[0].ref)
        return node_list
    
    