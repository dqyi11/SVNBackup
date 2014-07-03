from ExpandingMultiPartiteGraph import *
from Agent import *
from BacktrackingHeuristic import *
import copy

class MultiObjectiveBacktrackingPathPlanner(object):

    def __init__(self, hexamap, agent):
        self.map = hexamap
        self.runOnlyOnce = False
        self.agent = agent
        self.iterationCount = 0   
        
    def planPath(self, planGraph, start, planningLen, rewardDistributions, dimension):
        solutionGraph = ExpandingMultiPartiteGraph(planningLen, 'solutionGraph')
        solutionGraph.partitions[0].addVertex(start)
        
        backtracking = BacktrackingHeuristic(self.map, self.agent)
        
        estimatedFutureRewards = []
        for d in range(dimension):
            rewardDistribution = rewardDistributions[d]
            estimatedFutureReward = backtracking.getBacktrackedEstimation(planGraph, rewardDistribution)
            estimatedFutureRewards.append(estimatedFutureReward)
            
        
            
            
            
                
            
        