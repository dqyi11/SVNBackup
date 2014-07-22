import pygraphviz as pgv
import copy

class Partite(object):
    
    def __init__(self, index):
        self.index = index
        self.vertices = []
        self.edges = []
        self.startFrom = 0
        
    def addVertex(self, vertexId):
        if self.hasVertex(vertexId)==False:
            self.vertices.append(vertexId)  
        
    def getVertexIndex(self, vertexId):
        for i in range(len(self.vertices)):
            if self.vertices[i][0]==vertexId[0] and self.vertices[i][1]==vertexId[1]:
                return i
        return -1
    
    def hasVertex(self, vertexId):
        for v in self.vertices:
            if v[0]==vertexId[0] and v[1]==vertexId[1]:
                return True        
        return False
    
    def hasEdge(self, edgeId):
        for e in self.edges:
            if e[0][0]==edgeId[0][0] and e[0][1]==edgeId[0][1] and e[1][0]==edgeId[1][0] and e[1][1]==edgeId[1][1]:
                return True
        return False
    
    def addEdge(self, edgeId):
        if self.hasEdge(edgeId)==False:
            self.edges.append(edgeId)
            
    def findEdges(self, vertexId):
        edges = []
        for e in self.edges:
            if e[0][0]==vertexId[0] and e[0][1]==vertexId[1]:
                edges.append(e)
        return edges

class MultiPartiteGraph(object):
    
    def __init__(self, T, name):
        self.name = name
        self.T = T
        self.partitions = []
        for idx in range(self.T):
            p = Partite(idx)
            self.partitions.append(p)
        
    def updateState(self):
        startFrom = 0
        for t in range(self.T):
            self.partitions[t].startFrom = startFrom
            startFrom += len(self.partitions[t].vertices)
            
    def removeUnreachableVertices(self, t):
        if t==0:
            return
        tempVertices = copy.deepcopy(self.partitions[t].vertices)
        for v2 in tempVertices:
            found = False
            for e1 in self.partitions[t-1].edges:
                if e1[1][0]==v2[0] and e1[1][1]==v2[1]:
                    found = True
            if found==False:
                self.partitions[t].vertices.remove(v2)

        self.removeObsoleteEdges(t)
                
    def removeTerminatingVertices(self, t):
        if t==self.T-1:
            return
        tempVertices = copy.deepcopy(self.partitions[t].vertices)
        for v in tempVertices:
            found = False
            for e in self.partitions[t].edges:
                if e[0][0]==v[0] and e[0][1]==v[1]:
                    found = True
            if found==False:
                self.partitions[t].vertices.remove(v)

        self.removeObsoleteEdges(t-1)
    
    def cleanObsoleteEdges(self):
        for t in range(self.T):
            self.removeObsoleteEdges(t)
    
    def removeObsoleteEdges(self, t):
        if t < 0 or t >= self.T-1:
            return       
        tempEdges = copy.deepcopy(self.partitions[t].edges)
        for e1 in tempEdges:
            if False==self.partitions[t].hasVertex(e1[0]) or False==self.partitions[t+1].hasVertex(e1[1]):
                self.partitions[t].edges.remove(e1)
                
    def printString(self):
        for p in self.partitions:
            for v in p.vertices:
                print v
            for e in p.edges:
                print e
        
    def dump(self):        
        #self.printString()        
        #print "edges: " + str(len(self.edges))
        A = pgv.AGraph()
        for t1 in range(len(self.partitions)-1):
            t2 = t1+1
            p1 = self.partitions[t1]
            p2 = self.partitions[t2] 
            #print "start from " + str(p1.startFrom)           
            for edge in p1.edges:
                fromIdx = p1.getVertexIndex(edge[0])
                toIdx = p2.getVertexIndex(edge[1])
                if -1!=fromIdx and -1!=toIdx:
                    idxA = fromIdx+p1.startFrom
                    idxB = toIdx+p2.startFrom
                    #print str(t1) + " : " + str(idxA) + " - " + str(idxB)
                    A.add_edge(idxA, idxB)
        #print A.is_directed()
        A.write(self.name+".dot")
        B = pgv.AGraph(self.name+".dot")    
        B.layout(prog='dot')
        B.draw(self.name+".png")