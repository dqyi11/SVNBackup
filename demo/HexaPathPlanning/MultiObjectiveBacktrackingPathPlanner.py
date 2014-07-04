from ExpandingMultiPartiteGraph import *
from MultiObjectiveExpandingTree import *
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
        
        expandingTree = MultiObjectiveExpandingTree(planGraph, start, dimension)        
        backtracking = BacktrackingHeuristic(self.map, self.agent)
        
        estimatedFutureRewards = []
        for d in range(dimension):
            rewardDistribution = rewardDistributions[d]
            estimatedFutureReward = backtracking.getBacktrackedEstimation(planGraph, rewardDistribution)
            estimatedFutureRewards.append(estimatedFutureReward)
            
        for t in range(1,planningLen):
            for node in expandingTree.newNodeList:
                if node.state=="NEW" and node.level==t:
                    expandingTree.expandNode(node)
                    expandingTree.updateChidNodesInstantRewards(node, self.map, self.agent, rewardDistributions)
                    
                    
            
            
            
        
            
            
            
                
            
        