'''
Created on Nov 12, 2014

@author: daqing_yi
'''

from RRT import *

class RRG(RRT):

    def __init__(self, dimension, segmentLength):
        super(RRG, self).__init__(dimension, segmentLength)
        self.nearNodeNum = 10
        
    def expand(self):
        new_node = None
        while new_node == None:
            rndPos = self.generateRandomPos()
            nearest_node = self.findClosetNode(rndPos)
            
            # normalize along direction
            delta = [0.0,0.0]
            delta[0] = rndPos[0] - nearest_node.pos[0]
            delta[1] = rndPos[1] - nearest_node.pos[1]
            delta_len = np.sqrt(delta[0]**2+delta[1]**2)
            scale = self.segmentLength/float(delta_len)
            delta[0] = delta[0] * scale
            delta[1] = delta[1] * scale
            
            new_pos = [0, 0]
            new_pos[0] = nearest_node.pos[0] + int(delta[0])
            new_pos[1] = nearest_node.pos[1] + int(delta[1])
            
            if False == self.isCrossingObstacle(new_pos, nearest_node.pos):
                new_node = RRTNode(new_pos)
                new_node.cost = nearest_node.cost + self.segmentLength
                
                self.nodes.append(new_node)
                self.addEdge(nearest_node, new_node)
                self.addEdge(new_node, nearest_node)
                
                near_node_list = self.findNearVertices(new_node, self.nearNodeNum)
                for near_node in near_node_list:
                    if False == self.isCrossingObstacle(near_node.pos, new_node.pos):
                        self.addEdge(near_node, new_node)
                        self.addEdge(near_node, new_node)
                
    def findNearVertices(self, node, num):
        total_node_num = len(self.nodes)
        if num >= total_node_num:
            return self.nodes
        
        node_list = []
        dist_list = np.zeros(total_node_num, np.float)
        for i in range(total_node_num):
            dist_list[i] = np.sqrt((self.nodes[i].pos[0]-node.pos[0])**2+(self.nodes[i].pos[1]-node.pos[1])**2)
        
        sorted_indices = np.argsort(dist_list)
        for i in range(num):
            node_list.append(self.nodes[sorted_indices[i]])
        return node_list
        
        

        