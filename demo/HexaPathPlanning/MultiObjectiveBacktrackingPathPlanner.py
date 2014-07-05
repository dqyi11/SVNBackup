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
        self.terminalNodeList = []
        
    def planPath(self, planGraph, start, planningLen, rewardDistributions, dimension):
        
        expandingTree = MultiObjectiveExpandingTree(planGraph, start, dimension)        
        backtracking = BacktrackingHeuristic(self.map, self.agent)
        self.terminalNodeList = []
        
        estimatedFutureRewards = []
        for d in range(dimension):
            rewardDistribution = rewardDistributions[d]
            estimatedFutureReward = backtracking.getBacktrackedEstimation(planGraph, rewardDistribution)
            estimatedFutureRewards.append(estimatedFutureReward)
            
        for t in range(1,planningLen):
            for n1 in expandingTree.newNodeList:
                if n1.state=="NEW" and n1.level==t:
                    nondominated = True
                    for n2 in expandingTree.newNodeList:
                        if n2.state=="NEW" and n2.level==t:
                            if True==self.isDominated(n1, n2):
                                nondominated = False
                    if nondominated == False:
                        n1.state="Frozen"
                        expandingTree.newNodeList.remove(n1)
                        
                    if t==planningLen-1:
                        self.terminalNodeList.append(n1)
            
            if t<planningLen-1:
                for node in expandingTree.newNodeList:
                    if node.state=="NEW" and node.level==t:
                        expandingTree.expandNode(node)
                        expandingTree.updateChidNodesInstantRewards(node, self.map, self.agent, rewardDistributions)
            else:
                print "left " + str(len(expandingTree.newNodeList))

                
                    
    def isDominated(self, node1, node2):
        #if node1 dominated by node2
        dominated = True
        vecLen = node1.dimension
        same = True
        for i in range(vecLen):
            if node1.instantReward[i] + node1.maxTotalReward[i]> node2.instantReward[i] + node2.maxTotalReward[i]:
                dominated = False
            if node1.instantReward[i] + node1.maxTotalReward[i]!= node2.instantReward[i] + node2.maxTotalReward[i]:
                same = False
        if same==True:
            dominated = False
        return dominated   
                    
                    
            
            
            
        
            
            
            
                
            
        