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
        self.rrts_viz = BiRRTVisualizer(self.rrts)
        self.cost_func = cost_func
        
        self.pathMgr = PathManager(cost_func)
        
    def findPaths(self, start, goal, iterationNum, homotopyMgr):
        
        self.rrts.init(start, goal, self.cost_func, homotopyMgr)
        dividingRefs = homotopyMgr.getDividingRefs(start, goal)
        for dr in dividingRefs:
            self.rrts.dividingRefs.append([dr.open_seg[0], dr.open_seg[1]])
        
        for i in range(iterationNum):
            print "Iter@" + str(i)
            self.rrts.extend(self.rrts.st_kdtree_root, self.rrts.st_nodes)
            self.rrts.extend(self.rrts.gt_kdtree_root, self.rrts.gt_nodes)
            self.rrts_viz.update()
            
        paths, infos, costs = self.rrts.findPaths()
        
        for i in range(len(paths)):
            self.pathMgr.addPath(Path(paths[i], costs[i], infos[i]))
        
        self.rrts_viz.activePaths = paths
        
        #self.rrts_viz.update()
        
        return paths