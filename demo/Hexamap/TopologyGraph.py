import pygraphviz as pgv

class TopologyGraph(object):

    def __init__(self, directed, name):
        self.name = name
        self.directed = directed
        self.vertices = []
        self.edges = []
        
    def hasVertex(self, vertexId):
        for v in self.vertices:
            if v[0]==vertexId[0] and v[1]==vertexId[1]:
                return True        
        return False
    
    def getVertexIndex(self, vertexId):
        for i in range(len(self.vertices)):
            if self.vertices[i][0]==vertexId[0] and self.vertices[i][1]==vertexId[1]:
                return i
        return -1
        
    def hasEdge(self, edgeId):
        for e in self.edges:
            if e[0][0]==edgeId[0][0] and e[0][1]==edgeId[0][1] and e[1][0]==edgeId[1][0] and e[1][1]==edgeId[1][1]:
                return True
            if self.directed==False:
                if e[0][0]==edgeId[1][0] and e[0][1]==edgeId[1][1] and e[1][0]==edgeId[0][0] and e[1][1]==edgeId[0][1]:
                    return True
        return False            
        
    def addVertex(self, vertexId):
        if self.hasVertex(vertexId)==False:
            self.vertices.append(vertexId)        
        
    def addEdge(self, edgeId):
        if self.hasEdge(edgeId)==False:
            self.edges.append(edgeId)
            
    def dump(self):
        #print "edges: " + str(len(self.edges))
        A = pgv.AGraph()
        for edge in self.edges:
            idxA = self.getVertexIndex(edge[0])
            idxB = self.getVertexIndex(edge[1])
            A.add_edge(idxA, idxB)
            A.add_edge(idxB, idxA)
        #print A.is_directed()
        A.write(self.name+".dot")
        B = pgv.AGraph(self.name+".dot")    
        B.layout()
        B.draw(self.name+".png")
        
            
                    
    
        