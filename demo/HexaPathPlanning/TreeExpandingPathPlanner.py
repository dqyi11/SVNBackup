from ExpandingTree import *
from Agent import *
from BacktrackingHeuristic import *
import copy

def getMaxChildIndex(childList):
    maxVal = 0.0
    maxIdx = 0
    for i in range(len(childList)):
        if childList[i].maxTotalReward > maxVal:
            maxVal = childList[i].maxTotalReward
            maxIdx = i
    return maxIdx 

class TreeExpandingPathPlanner(object):
    
    def __init__(self, hexamap, agent):
        self.map = hexamap
        self.runTime = 1
        self.agent = agent        
        
    def planPath(self, planGraph, start, planningLen, rewardDistribution):
        solutionPath = []
        solutionPath.append(start)
        
        backtracking = BacktrackingHeuristic(self.map, self.agent)
        expandingTree = ExpandingTree(planGraph, start)
        
        currentNode = expandingTree.root
        for t in range(1,planningLen):
            pos = currentNode.pos
            expandingTree.expandNode(currentNode)
            
            rewardDistribution = self.agent.applyObservation(pos, self.map, rewardDistribution)        
            maxTotalRewards = backtracking.getMaximumTotalReward(planGraph, rewardDistribution, solutionPath)
            
            
            for node in currentNode.childNodeList:
                forIdx = planGraph.partitions[t].getVertexIndex(node.pos)
                node.maxTotalReward = maxTotalRewards[forIdx]
            maxNextIdx = getMaxChildIndex(currentNode.childNodeList)
            print maxTotalRewards
            print "T:" + str(t) + "  - " + str(maxNextIdx)
            currentNode = currentNode.childNodeList[maxNextIdx]
            solutionPath.append(currentNode.pos)
        
        return solutionPath
    

        