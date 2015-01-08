'''
Created on Jan 3, 2015

@author: daqing_yi
'''

from RRT import *

class RRG(RRT):

    def __init__(self, sampling_range, segmentLength):
        super(RRG, self).__init__(sampling_range, segmentLength)
        self.nearNodeNum = 3
        self.gamma = 1.0
        self.radius = self.segmentLength
        
    def init(self, start, goal, costFunc):
        super(RRG, self).init(start, goal)
        self.costFunc = costFunc
        self.root.cost = self.costFunc(self.root, None)
        
    def extend(self):
        new_node = None
        while new_node == None:
            rndPos = self.sampling()
            nearest_node = self.findNearestNeighbor(rndPos)
            
            new_pos = self.steer(rndPos, nearest_node.pos)
            
            if True == self.isObstacleFree(nearest_node.pos, new_pos):
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
                            
    def calcRadius(self): 
        vertexNum = len(self.nodes)
        self.radius = np.power( ( self.gamma * np.log(float(vertexNum+1)) / (float(vertexNum+1)) ) ,  1.0/self.dimension) 
        #self.radius = np.min([self.radius , self.segmentLength])
        
        return self.radius                
                
    def findNearVertices(self, pos, num):
        node_list = []
        results = self.kdtree_root.search_knn(pos, num)
        '''
        self.calcRadius()
        print self.radius
        results = self.kdtree_root.search_nn_dist(pos, self.radius)
        for res in results:
        '''
        for res, dist in results:
            if res.data[0]==pos[0] and res.data[1]==pos[1]:
                continue
            node_list.append(res.ref)
        return node_list
    
    def findPath(self):
        path = []
        
        
        start_pos = np.zeros(2)
        goal_pos = np.zeros(2)
        start_pos[0], start_pos[1] = self.start[0], self.start[1]
        goal_pos[0], goal_pos[1] = self.goal[0], self.goal[1]
        
        nearest_to_goal = self.findNearestNeighbor(goal_pos)
        
        import networkx as nx
        G = nx.Graph()
        for n in self.nodes:
            for c in n.children:
                from_node_str = "["+str(int(n.pos[0]))+","+str(int(n.pos[1]))+"]"
                to_node_str = "["+str(int(c.pos[0]))+","+str(int(c.pos[1]))+"]"
                dist = self.costFunc(n, c)
                G.add_edge(from_node_str, to_node_str, distance=dist)
                
        start_node_str = "["+str(int(start_pos[0]))+","+str(int(start_pos[1]))+"]"
        end_node_str = "["+str(int(nearest_to_goal.pos[0]))+","+str(int(nearest_to_goal.pos[1]))+"]"
        node_list = nx.shortest_path(G, start_node_str, end_node_str, 'distance')
            
        import os
        for n in node_list:
            x = None
            exec "x="+n
            path.append(x)
        path.append([goal_pos[0],goal_pos[1]])
        
        return path
    
    