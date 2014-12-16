'''
Created on Dec 13, 2014

@author: daqing_yi
'''

import sys
import numpy as np
import copy
from KDTree import *
from World import *

class Vertex(object):
    
    def __init__(self):
        self.parent = None
        self.state = None
        self.trajFromParent = None
        
        self.costFromParent = 0.0
        self.costFromRoot = 0.0
        
        self.children = []
        
    def getState(self):
        return self.state
        
    def getParent(self):
        return self.parent
        
    def getCost(self):
        return self.costFromRoot

class RRTstarPlanner(object):


    def __init__(self, dimension, gamma=1.0):
        self.gamma = gamma
        self.lowerBoundCost = sys.float_info.max
        self.lowerBoundVertex = None
        
        self.dimension = dimension
        self.kdtree = None
        self.root = None
        self.numVertices = 0
        self.world = None
        
        self.numVertices = 0
        self.listVertices = []
        
        # Initialize the root vertex
        self.root = Vertex()
        self.root.state = np.zeros(self.dimension)
        self.root.costFromParent = 0.0
        self.root.costFromRoot = 0.0
        self.root.trajFromParent = None

        
    def insertIntoKDTree(self, vertexIn):
        stateKey = self.world.getStateKey(vertexIn.state)
        self.kdtree.insert(stateKey, vertexIn)
        
    def getNearestVertex(self, stateIn):
        # Get the state key for the query state
        stateKey = self.world.getStateKey(stateIn)
        
        # Search the kdtree for the nearest vertex
        kdres = self.kdtree.findNearest(stateKey)
        if kdres.isEnd():
            vertexPointerOut = None
        vertexPointerOut = kdres.itemData()
        
        return vertexPointerOut
        
        
    def getNearVertices(self, stateIn):
        # Get the state key for the query state
        stateKey = self.world.getStateKey(stateIn)
        
        # Compute the ball radius
        tVal = np.log(self.numVertices+1.0) / (self.numVertices)
        ballRadius = self.gamma * np.power(tVal,1.0/self.dimension)
        
        # Search kdtree for the set of near vertices
        kdres = self.kdtree.findNearestRange(stateKey, ballRadius)
        
        # Create the vector data structure for storing the results
        numNearVertices = kdres.getSize()
        if numNearVertices==0:
            vectorNearVerticesOut = []
            return vectorNearVerticesOut
        
        vectorNearVerticesOut = []
        # Place pointers to the near vertices into the vector
        idx = 0
        kdres.rewind()
        while kdres.isEnd()==False:
            vertexCurr = kdres.itemData()
            vectorNearVerticesOut.append(vertexCurr)
            kdres.next()
            idx += 1
            
        return vectorNearVerticesOut
            
    def checkUpdateBestVertex(self, vertexIn):
        
        if self.world.isReachingTarget(vertexIn.getState()):
            costCurr = vertexIn.getCost()
            if self.lowerBoundVertex == None or ( (self.lowerBoundVertex != None) and (costCurr < self.lowerBoundCost) ):
                self.lowerBoundVertex = vertexIn
                self.lowerBoundCost = costCurr
        
        
    def insertTrajectory(self, vertexStartIn, trajectoryIn):
        
        # Check for admissible cost-to-go
        if self.lowerBoundVertex != None:
            costToGo = self.world.evaluateCostToGo(trajectoryIn.getEndState())
            if costToGo >= 0.0:
                if self.lowerBoundCost < vertexStartIn.getCost() + costToGo:
                    return None
        
        # Create a new end vertex
        vertexNew = Vertex()
        vertexNew.state = np.zeros(self.dimension)
        vertexNew.parent = None
        vertexNew.state = trajectoryIn.getEndState()
        self.insertIntoKDTree(vertexNew)
        self.listVertices.append(vertexNew)
        self.numVertices += 1
        
        # Insert the trajectory between the start and end vertices
        self.insertVertexToTrajectory(vertexStartIn, trajectoryIn, vertexNew)
                
        return vertexNew
        
    def insertVertexToTrajectory(self, vertexStartIn, trajectoryIn, vertexEndIn):
        
        # Update the costs
        vertexEndIn.costFromParent = trajectoryIn.evaluateCost()
        vertexEndIn.costFromRoot = vertexStartIn.costFromRoot + vertexEndIn.costFromParent
        self.checkUpdateBestVertex(vertexEndIn)
        
        # Update the trajectory between the two vertices
        vertexEndIn.trajFromParent = copy.deepcopy(trajectoryIn)
            
        # Update the parent to the end vertex
        vertexEndIn.parent = vertexStartIn
        
        # Add the end vertex to the set of children
        vertexStartIn.children.append(vertexEndIn)
        

        
    def getRootVertex(self):
        
        return self.root
    
    def initialize(self):
        if self.world == None:
            return False
        
        # Backup the root
        if self.root != None:
            rootBackup = copy.deepcopy(self.root)
        
        # Delete all the vertices
        self.listVertices = []
        self.numVertices = 0
        self.lowerBoundCost = sys.float_info.max
        self.lowerBoundVertex = None
        
        # Clear the kdtree
        self.kdtree = KDTree(self.dimension)
        
        # Initialize the variables
        self.root = rootBackup
        if self.root!= None:
            self.listVertices.append(self.root)
            self.insertIntoKDTree(self.root)
            self.numVertices += 1
        self.lowerBoundCost = sys.float_info.max
        self.lowerBoundVertex = None
        
    def setGamma(self, gamma):
        self.gamma = gamma
        
    def compareVertexCostPairs(self, i, j):
        return i.second < j.second
        
    def findBestParent(self, stateIn, vectorNearVerticesIn):
        # Compute the cost of extension for each near vertex
        numNearVertices = len(vectorNearVerticesIn)
        vertexBest = None
        
        vectorVertexCostPairs = []
        for i in range(numNearVertices):
            trajCost = self.world.evaluateExtensionCost(vectorNearVerticesIn[i].state, stateIn)
            vectorVertexCostPairs.append((vectorNearVerticesIn[i], vectorNearVerticesIn[i].costFromRoot + trajCost))
            
        # Sort vertices according to cost
        sorted(vectorVertexCostPairs, key=lambda item: item[1], reverse=False)
        
        # Try out each extension according to increasing cost
        connectionEstablished = False
        for i in range(len(vectorVertexCostPairs)):
            vertexCurr = vectorVertexCostPairs[i][0]
            # Extend the current vertex towards stateIn
            # (and this time check for collision with obstacles)
            
            trajectoryOut, ret = self.world.extendTo(vertexCurr.state, stateIn)
            if ret > 0:
                vertexBest = vertexCurr
                connectionEstablished = True
                break
            
        # Return success if a connection was established
        if connectionEstablished:
            return vertexBest, trajectoryOut, True
        
        return vertexBest, trajectoryOut, False
            
        
    def updateBranchCost(self, vertexIn, depth):
        # Update the cost for each children
        children_num = len(vertexIn.children)
        for i in range(children_num):
            vertex = vertexIn.children[i]
            vertex.costFromRoot = vertexIn.costFromRoot + vertex.costFromParent
            self.checkUpdateBestVertex(vertex)
            self.updateBranchCost(vertex, depth+1)
        
        
    def rewireVertices(self, vertexNew, vectorNearVertices): 
        # Repeat for all vertices in the set of near vertices
        for vertexCurr in vectorNearVertices:
            
            # Check whether the extension results in an exact connection
            costCurr = self.world.evaluateExtensionCost(vertexNew.state, vertexCurr.state )
            if costCurr < 0:
                continue
            
            # Check whether the cost of the extension is smaller than current cost
            totalCost = vertexNew.costFromRoot + costCurr
            if totalCost < vertexCurr.costFromRoot + costCurr:
                # Compute the extension (checking for collision)
                trajectory, ret = self.world.extendTo(vertexNew.state, vertexCurr.state)
                if ret == False:
                    continue
                
                # Insert the new trajectory to the tree by rewiring
                self.insertVertexToTrajectory(vertexNew, trajectory, vertexCurr)
                
                # Update the cost of all vertices in the rewired branch
                self.updateBranchCost(vertexCurr, 0)
        
    def iteration(self):
        
        # 1. Sample a new state
        stateRandom = self.world.sampleState()

        # 2. Compute the set of all near vertices
        vectorNearVertices = self.getNearVertices(stateRandom)
        
        # 3. Find the best parent and extend from that parent
        vertexParent = None
        
        if len(vectorNearVertices) == 0:
            
            # 3.a Extend the nearest
            vertexParent = self.getNearestVertex(stateRandom)
            if vertexParent == None:
                return False
            ret, trajectory = self.world.extendTo(vertexParent.state, stateRandom)
            if ret == False:
                return False
        else:
            # 3.b Extend the best parent within the near vertices
            vertexParent, trajectory, ret = self.findBestParent(stateRandom, vectorNearVertices)
            if ret == False:
                return False
        
        # 3.c add the trajectory from the best parent to the tree
        vertexNew = self.insertTrajectory(vertexParent, trajectory)
        if vertexNew == None:
            return False
        
        # 4. Rewire the tree 
        if len(vectorNearVertices) > 0:
            self.rewireVertices(vertexNew, vectorNearVertices)
            
        return True
        
    def getBestTrajectory(self):
        
        trajectoryOut = []
        if self.lowerBoundVertex == None:
            return trajectoryOut, False
        
        vertexCurr = self.lowerBoundVertex
        
        while vertexCurr!= None:
            stateCurr = vertexCurr.state
            
            stateArrCurr = np.zeros(2)
            stateArrCurr[0] = stateCurr[0]
            stateArrCurr[1] = stateCurr[1]
            
            trajectoryOut.push_front(stateArrCurr)
            
            vertexParent = vertexCurr.getParent()
            
            if vertexParent != None:
                stateParent = vertexParent.state
                
                trajectory = self.world.getTrajectory(stateParent, stateCurr)
                
                trajectory.reverse()
                for stateArrFromParentCurr in trajectory:
                    
                    stateArrCurr = np.zeros(2)
                    stateArrCurr[0] = stateArrFromParentCurr[0]
                    stateArrCurr[1] = stateArrFromParentCurr[1]
                    trajectoryOut.append(stateArrCurr)
                    
            vertexCurr = vertexParent 
            
        return trajectoryOut, True
