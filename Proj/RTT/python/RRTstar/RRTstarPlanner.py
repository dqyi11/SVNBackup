'''
Created on Dec 13, 2014

@author: daqing_yi
'''

class Vertex(object):
    
    def __init__(self):
        self.parent = None
        self.state = None
        self.children = []
        self.costFromParent = 0.0
        self.costFromRoot = 0.0
        self.trajFromParent = []
        
    def getState(self):
        return self.state
        
    def getParent(self):
        return self.parent
        
    def getCost(self):
        return self.costFromRoot

class RRTstarPlanner(object):


    def __init__(self, dimensin, gamma):
        self.gamma = 1.0
        self.lowerBoundCost = 
        self.lowerBoundVertex = None
        
        self.kdtree = None
        self.root = None
        self.numVertices = 0
        self.world = None
        
    def insertIntoKDTree(self, vertexIn):
        stateKey = self.world.getStateKey(vertexIn.state)self
        self.kdtree.insert(stateKey, vertexIn)
        
    def getNearestVertex(self, stateIn, vertexPointerOut):
        # Get the state key for the query state
        
        # Search the kdtree for the nearest vertex
        
        
    def getNearVertices(self, stateIn):
        # Get the state key for the query state
        
        # Compute the ball radius
        
        # Search kdtree for the set of near vertices
        
        # Create the vector data structure for stroing the results
        
        # Place pointers to the near vertices into the vector
        
    def checkUpdateBestVertex(self, vertexIn):
        
        
    def insertTrajectory(self, vertexStartIn, trajectoryIn):
        
        # Check for admissible cost-to-go
        
        # Create a new end vertex
        
        # Insert the trajectory between the start and end vertices
        
    def insertTrajectory(self, vertexStartIn, trajectorIn, vertexEndIn):
        
        # Update the costs
        
        # Update the trajectory between the two vertices
        
        # Update the parent to the end vertex
        
        # Add the end vertex to the set of children
        
    def setWorld(self, worldIn):
        
        self.dimension = worldIn.dimension
        self.world = worldIn
        
    def getRootVertex(self):
        
        return self.root
    
    def initialize(self):
        if self.world = None:
            return False
        
        # Backup the root
        
        # Delete all the vertices
        
        # Clear the kdtree
        
        # Initialize the variables
        
        
    def setGamma(self, gamma):
        self.gamma = gamma
        
        
    def compareVertexCostPairs(self, i, j):
        
        
    def findBestParent(self, ):
        
        # Compute the cost of extension for each near vertex
        
        # Sort vertices according to cost
        
        # Try out each extension according to increasing cost
        
    def updateBranchCost(self):
        
        # Update the cost for each children
        
    def rewireVertices(self):
        
        # Repeat for all vertices in the set of near vertices
        
    def iteration(self):
        
        # 1. Sample a new state

        # 2. Compute the set of all near vertices
        
        # 3. Find the best parent and extend from that parent
        
        # 3.a Extend the nearest
        
        # 3.b Extend the best parent within the near vertices
        
        # 3.c add the trajectory from the best parent to the tree
        
        # 4. Rewire the tree  
        
    def getBestTrajectory(self):
        
        
        
        
        
        
        
        
        
        
        
        
        

        