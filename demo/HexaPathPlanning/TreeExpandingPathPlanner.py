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
        self.runOnlyOnce = False
        self.agent = agent
        self.iterationCount = 0        
        
    def planPath(self, planGraph, start, planningLen, rewardDistribution):
        solutionPath = []
        solutionPath.append(start)
        
        backtracking = BacktrackingHeuristic(self.map, self.agent)
        expandingTree = ExpandingTree(planGraph, start)
        
        bestFoundSolution = None
        bestFoundSolutionReward = -0.1       
        
        currentNode = expandingTree.root
        self.iterationCount = 0   
        stopCriteria = False
        
        while stopCriteria==False:
            tempRewardDistribution = copy.deepcopy(rewardDistribution)
            startIdx = len(solutionPath)
            
            #print "before search-current pos: " + str(currentNode.pos)            
            #print "before search-solution path: " + str(solutionPath)
            
            
            for t in range(startIdx,planningLen):
                pos = currentNode.pos
                expandingTree.expandNode(currentNode)
                #print "@" + str(t) + " expand " + str(currentNode.pos)
                expandingTree.updateChidNodesInstantRewards(currentNode, self.map, self.agent, tempRewardDistribution)
                
                rewardDistribution = self.agent.applyObservation(pos, self.map, tempRewardDistribution)        
                maxTotalRewards = backtracking.getMaximumTotalReward(planGraph, tempRewardDistribution, solutionPath)
                
                for node in currentNode.childNodeList:
                    forIdx = planGraph.partitions[t].getVertexIndex(node.pos)
                    node.maxTotalReward = maxTotalRewards[forIdx]
                maxNextIdx = getMaxChildIndex(currentNode.childNodeList)
                #print maxTotalRewards
                #print "T:" + str(t) + "  - " + str(maxNextIdx)
                currentNode = currentNode.childNodeList[maxNextIdx]
                solutionPath.append(currentNode.pos)
                
            pathReward = self.agent.getPathReward(solutionPath, self.map, rewardDistribution)
            
            if pathReward > bestFoundSolutionReward:
                bestFoundSolution = solutionPath
                bestFoundSolutionReward = pathReward
            
            self.iterationCount += 1
            if self.runOnlyOnce==True:
                stopCriteria=True
                
            currentNode = expandingTree.getMaxNewNode()
            if currentNode != None:
                solutionPath = expandingTree.getSubpath(currentNode)
            else:
                solutionPath = []

            #print bestFoundSolution
            #if currentNode != None:
                #print "after search-current pos: " + str(currentNode.pos)            
                #print "after search-solution path: " + str(solutionPath)
                
            if currentNode == None:
                stopCriteria=True
        
        return bestFoundSolution
    

        