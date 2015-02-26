'''
Created on Jan 4, 2015

@author: daqing_yi
'''

from BiRRTstar import *
from BiRRTVisualizer import *
from PathManager import *

class BiRRTstarPlanner(object):
    
    def __init__(self, planning_range, segment_length, cost_func=None, map_file=None):
        self.planningRange = planning_range
        self.segmentLength = segment_length
        self.mapFile = map_file
        self.rrts = BiRRTstar(self.planningRange, self.segmentLength)
        if self.mapFile!= None:
            self.rrts.loadMap(self.mapFile)
            
        self.pathMgr = PathManager(cost_func)
        self.rrts_viz = BiRRTVisualizer(self.rrts, self.pathMgr)
        self.cost_func = cost_func
        
        
    def connectPath(self, new_s_node, new_g_node, rrts):
        minPath = None        

        if new_s_node == None:
            
            near_node_list = rrts.findNearVertices(rrts.st_kdtree_root, new_g_node.pos)
            
            infoVec = []
            for near_node in near_node_list:
                if rrts.isObstacleFree(new_g_node.pos, near_node.pos):
                    delta_cost = self.cost_func(new_g_node.pos, near_node.pos)
                    total_cost = new_g_node.cost + near_node.cost + delta_cost
                    infoVec.append((near_node, total_cost))
            
            if len(infoVec) > 0:
                infoVec.sort( key=lambda x: x[1], reverse=False )
                
                min_cost = infoVec[0][1]
                min_node = infoVec[0][0]
                min_path, min_stringbit = self.getPathFromTwoNodes(min_node, new_g_node, self.rrts)
                
                minPath = Path(min_path, min_cost, min_stringbit)
 
        elif new_g_node == None:
            
            near_node_list = rrts.findNearVertices(rrts.gt_kdtree_root, new_s_node.pos)
            
            infoVec = []
            for near_node in near_node_list:
                if rrts.isObstacleFree(new_s_node.pos, near_node.pos):
                    delta_cost = self.cost_func(new_s_node.pos, near_node.pos)
                    total_cost = new_s_node.cost + near_node.cost + delta_cost
                    infoVec.append((near_node, total_cost))
                    
            if len(infoVec) > 0:
                infoVec.sort( key=lambda x: x[1], reverse=False )       
                
                min_cost = infoVec[0][1]
                min_node = infoVec[0][0]
                min_path, min_stringbit = self.getPathFromTwoNodes(new_s_node, min_node, self.rrts)
                
                minPath = Path(min_path, min_cost, min_stringbit)
  
        return minPath
    
    def getPathFromTwoNodes(self, node_s, node_g, rrts):
        path = []
        stringbit = []
        
        if node_s == None or node_g == None:
            return path, stringbit
        
        if rrts.isObstacleFree(node_s.pos, node_g.pos) == False:
            return path, stringbit
        
        subpathFromStart = rrts.getSubNodeList(node_s, rrts.st_root)
        subpathFromGoal = rrts.getSubNodeList(node_g, rrts.gt_root)
        
        crossInt = self.rrts.homotopyMgr.world_map.getCrossingSubsegment(node_s.pos, node_g.pos)
        
        for p in reversed(subpathFromStart):
            path.append(p.pos)
        for p in subpathFromGoal:
            path.append(p.pos)
            
        if node_s.strBit != None:   
            for c in node_s.strBit:
                stringbit.append(c)
        if crossInt != None:
            stringbit.append(crossInt)
        if node_g.strBit != None:
            for c in reversed(node_g.strBit):
                stringbit.append(c)
        
        return path, stringbit
        
    def findPaths(self, start, goal, iterationNum, homotopyMgr):
        
        self.rrts.init(start, goal, self.cost_func, homotopyMgr)
        dividingRefs = homotopyMgr.getDividingRefs(start, goal)
        for dr in dividingRefs:
            self.rrts.dividingRefs.append([dr.open_seg[0], dr.open_seg[1]])
        
        for i in range(iterationNum):
            print "Iter@" + str(i)
            new_s_node = self.rrts.extend(self.rrts.st_kdtree_root, self.rrts.st_nodes)
            new_g_node = self.rrts.extend(self.rrts.gt_kdtree_root, self.rrts.gt_nodes)
        
            min_s_path = self.connectPath(new_s_node, None, self.rrts)
            min_g_path = self.connectPath(None, new_g_node, self.rrts)
            
            if min_s_path != None:
                self.pathMgr.importPath(min_s_path)
            if min_g_path != None:
                self.pathMgr.importPath(min_g_path)
            
            self.rrts_viz.currentPaths = self.pathMgr.getPaths()
            
            self.rrts_viz.update()
            
        self.rrts_viz.currentPaths = []
        
        return self.pathMgr.getPaths()