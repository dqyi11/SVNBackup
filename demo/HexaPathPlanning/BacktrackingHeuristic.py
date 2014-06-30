import numpy as np
import copy

class BacktrackingHeuristic(object):
    
    def __init__(self, hexamap, agent):
        self.hexamap = hexamap
        self.agent = agent
    
    def getMaximumTotalReward(self, planningGraph, rewardDistribution, subpath):
        subpathLen = len(subpath)
        planningLen = planningGraph.T
        
        #rewardArrayNum = len(planningGraph.partitions[subpathLen].vertices)
        
        backtrackingMatrix = []
        for t in range(planningLen):
            vexNumInPartite = len(planningGraph.partitions[t].vertices)
            backtrackingMatrix.append(np.zeros(vexNumInPartite))
        
        #print range(planningLen-1, subpathLen-1, -1)
        for t in range(planningLen-1, subpathLen-1, -1):
            vexNumInPartite = len(planningGraph.partitions[t].vertices)
            if t==planningLen-1:
                for i in range(vexNumInPartite):
                    pos = planningGraph.partitions[t].vertices[i]
                    backtrackingMatrix[t][i] = self.agent.getObservation(pos, self.hexamap, rewardDistribution)
            else:
                for i in range(vexNumInPartite):
                    vex = planningGraph.partitions[t].vertices[i]
                    pos = vex
                    edges = planningGraph.partitions[t].findEdges(vex)
                    maxFutureVal = -0.1
                    for e in edges:
                        nextIdx = planningGraph.partitions[t+1].getVertexIndex(e[1])
                        if nextIdx != -1:
                            currentVal = backtrackingMatrix[t+1][nextIdx]
                            if maxFutureVal < currentVal:
                                maxFutureVal = currentVal
                    backtrackingMatrix[t][i] = self.agent.getObservation(pos, self.hexamap, rewardDistribution)+maxFutureVal
                    #print "B:" + str(backtrackingMatrix[t][i])
        #maximumTotalReward = copy.deepcopy(backtrackingMatrix[subpathLen])        
        #return maximumTotalReward
        return backtrackingMatrix[subpathLen]
    
    
        
        
        