from MultiPartiteGraph import *

class PlanningPathGenerator(object):
    
    def __init__(self, hexamap):        
        self.hexamap = hexamap
        
    def generatePlanningPathGraph(self, humanPath, radius, name):
        pathLen = len(humanPath)
        
        graph = MultiPartiteGraph(pathLen, name)
        #print "radius: " + str(radius)
        for t in range(pathLen):
            if t==0:
                graph.partitions[t].vertices.append(humanPath[t])
            else:                
                hexes = self.hexamap.getHexesByRadius(humanPath[t], radius)
                #print "hex num: " + str(len(hexes))
                for hx in hexes:
                    graph.partitions[t].addVertex(hx)
                    
        graph.updateState()    
                
        for t1 in range(pathLen-1):
            t2 = t1+1
            for v1 in graph.partitions[t1].vertices:
                for v2 in graph.partitions[t2].vertices:
                    if True == self.hexamap.isConnected(v1, v2):
                        graph.partitions[t1].addEdge((v1, v2))
                        
        return graph
    
    def forwardPrune(self, graph):
        for t in range(1, graph.T):
            print t
            graph.removeUnreachableVertices(t)
        graph.cleanObsoleteEdges()
        
        graph.updateState()
        return graph
        
    def backwardPrune(self, graph):
        for t in range(graph.T-2, -1, -1):
            graph.removeTerminatingVertices(t)
        graph.cleanObsoleteEdges()
        
        graph.updateState()
        return graph