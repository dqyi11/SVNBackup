import numpy as np
import copy

class MultiObjectiveExpandingNode(object):
    
    def __init__(self, pos, level, index, dimension):
        self.pos = pos
        self.level = level
        self.index = index
        self.state = "NEW"
        self.parentNode = None
        self.childNodeList = []
        self.dimension = dimension
        self.maxTotalReward = np.zeros(dimension)
        self.instantReward = np.zeros(dimension)
    

class MultiObjectiveExpandingTree(object):
    
    def __init__(self, graph, rootPos, dimension):
        self.graph = graph
        self.T = graph.T
        self.dimension = dimension        
        self.root = MultiObjectiveExpandingNode(rootPos, 0, 0, self.dimension)
        self.newNodeList = []
        self.newNodeList.append(self.root)
        
    def expandNode(self, node):
        if node.state != "NEW":
            return        
        pos = node.pos
        level = node.level        

        self.newNodeList.remove(node)        
        
        partite = self.graph.partitions[level]
        nextLevel = level + 1
        nextPartite = self.graph.partitions[nextLevel]
        edges = partite.findEdges(pos)
        for e in edges:
            nextIdx = nextPartite.getVertexIndex(e[1])
            childNode = MultiObjectiveExpandingNode(e[1], nextLevel, nextIdx, self.dimension)
            childNode.parentNode = node
            node.childNodeList.append(childNode)
            if childNode.level==self.T-1:
                childNode.state = "EXPANDED"
            else:
                self.newNodeList.append(childNode)
        self.state = "EXPANDED"
            
    def updateChidNodesInstantRewards(self, node, hexamap, agent, rewardDistributions):
        subpath = self.getSubpath(node)
        subpathScore = agent.getPathRewardVec(subpath, hexamap, rewardDistributions)
        for childNode in node.childNodeList:
            instantReward = agent.getObservationVec(childNode.pos, hexamap, rewardDistributions)        
            for d in range(self.dimension):
                childNode.instantReward[d] = subpathScore[d] + instantReward[d]
            
                        
    def getSubpath(self, node):
        path = []
        for t in range(node.level+1):
            path.append([0,0])
        
        currentNode = node
        currentLevel = node.level
        while currentNode!=None:
            path[currentLevel][0] = currentNode.pos[0]
            path[currentLevel][1] = currentNode.pos[1]
            currentNode = currentNode.parentNode
            currentLevel -= 1        
        return path
    
        
                      
        
        