from ExpandingMultiPartiteGraph import *
from Agent import *
from BacktrackingHeuristic import *
import copy

class MultiObjectivePathPlanner(object):

    def __init__(self, self, hexamap, agent):
        self.map = hexamap
        self.runOnlyOnce = False
        self.agent = agent
        self.iterationCount = 0   
        
    def planPath(self, planGraph, start, planningLen, rewardDistribution):
        solutionGraph = ExpandingMultiPartiteGraph(planningLen, 'solutionGraph')
        solutionGraph.partitions[0].addVertex(start)
        
        backtracking = BacktrackingHeuristic(self.map, self.agent)
        
        for t in range(1,planningLen):
            
        