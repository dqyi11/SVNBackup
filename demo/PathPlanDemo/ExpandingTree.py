from MultiPartiteGraph import *

class ExpandingNode(object):
    
    def __init__(self, pos, level, index):
        self.pos = pos
        self.level = level
        self.index = index
        self.state = "NEW"
        self.parentNode = None
        self.childNodeList = []
        self.maxTotalReward = 0.0
        self.parentReward = 0.0

class ExpandingTree(object):
    
    def __init__(self, graph, rootPos):
        self.graph = graph
        self.T = graph.T        
        self.root = ExpandingNode(rootPos, 0, 0)
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
            childNode = ExpandingNode(e[1], nextLevel, nextIdx)
            childNode.parentNode = node
            node.childNodeList.append(childNode)
            if childNode.level==self.T-1:
                childNode.state = "EXPANDED"
            else:
                self.newNodeList.append(childNode)
        self.state = "EXPANDED"
            
    def updateChidNodesInstantRewards(self, node, hexamap, agent, rewardDistribution):
        tempRewardDist = copy.deepcopy(rewardDistribution)
        subpath = self.getSubpath(node)
        subpathScore = agent.getPathReward(subpath, hexamap, tempRewardDist)
        tempRewardDist = agent.applyObservation(node.pos, hexamap, tempRewardDist)
        for childNode in node.childNodeList:
            childNode.parentReward = subpathScore# + agent.getObservation(childNode.pos, hexamap, tempRewardDist)        
            
    def getMaxNewNode(self):
        maxNode = None
        maxNodeVal = -0.1        
        for node in self.newNodeList:
            if node.state == "NEW":
                if node.maxTotalReward+node.parentReward > maxNodeVal:
                    maxNodeVal = node.maxTotalReward+node.parentReward
                    maxNode = node                    
        return maxNode
    
    def freeze(self, freezeThreshold):
        for node in self.nodeList:
            if node.state=="NEW":
                if node.maxTotalReward+node.parentReward <= freezeThreshold:
                    node.state = "FROZEN"
                        
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
            
    
    