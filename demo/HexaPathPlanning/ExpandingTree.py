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

class ExpandingTree(object):
    
    def __init__(self, graph, rootPos):
        self.graph = graph
        self.T = graph.T
        self.nodeList = []
        
        self.root = ExpandingNode(rootPos, 0, 0)
        self.nodeList.append(self.root)
                
    def expandNode(self, node):
        if node.state != "NEW":
            return        
        pos = node.pos
        level = node.level        
        if level==self.T-1:
            self.state = "EXPANDED"
        else:
            partite = self.graph.partitions[level]
            nextLevel = level + 1
            nextPartite = self.graph.partitions[nextLevel]
            edges = partite.findEdges(pos)
            for e in edges:
                nextIdx = nextPartite.getVertexIndex(e[1])
                childNode = ExpandingNode(e[1], nextLevel, nextIdx)
                childNode.parentNode = node
                node.childNodeList.append(childNode)
                self.nodeList.append(childNode)
            self.state = "EXPANDED"
            
    def getMaxNewNode(self):
        maxNode = None
        maxNodeVal = -0.1
        
        for node in self.nodeList:
            if node.state == "NEW":
                if node.maxTotalReward > maxNodeVal:
                    maxNodeVal = node.maxTotalReward
                    maxNode = node                    
        return node
    
    def freeze(self, freezeThreshold):
        for node in self.nodeList:
            if node.state=="NEW":
                if node.maxTotalReward <= freezeThreshold:
                    node.state = "FROZEN"
            
            
                
            
    
    