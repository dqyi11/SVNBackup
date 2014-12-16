'''
Created on Dec 13, 2014

@author: daqing_yi
'''
import numpy as np
import copy 

DISCREMINZATION_STEP = 0.01

class Region(object):
    
    def __init__(self, dimensionNum):
        self.dimensionNum = dimensionNum
        self.center = np.zeros(self.dimensionNum)
        self.size = np.zeros(self.dimensionNum)


class Trajectory(object):
    
    def __init__(self):
        self.endState = None
        self.totalVariation = 0.0
        self.vertices = []
        
    def getEndState(self):
        return self.endState
    
    def evaluateCost(self):
        return self.totalVariation
        
class World(object):
    
    def __init__(self, dimensionNum):
        self.dimensionNum = dimensionNum
        self.regionOperating = Region(self.dimensionNum)
        self.regionGoal = Region(self.dimensionNum)
        self.obstacles = []
        self.rootState = np.zeros(self.dimensionNum)
        
    def getStateKey(self, stateIn):
        stateKey = np.zeros(self.dimensionNum)
        for i in range(self.dimensionNum):
            stateKey[i] = stateIn[i] / self.regionOperating.size[i]
        return stateKey  
        
    def isReachingTarget(self, stateIn):
        for i in range(self.dimensionNum):
            if np.abs(stateIn[i] - self.regionGoal.center[i]) > self.regionGoal.size[i]/2 :
                return False
        return True
        
    def isInCollision(self, stateIn):
        for obs in self.obstacles:
            collisionFound = True
            for i in range(self.dimensionNum):
                if np.abs(obs.center[i]-stateIn[i]) > obs.size[i]:
                    collisionFound = False
                    break
            if collisionFound:
                return True
        return False        
        
    def sampleState(self):
        randomStateOut = np.zeros(self.dimensionNum)
        randomVal = np.random.random(self.dimensionNum)
        for i in range(self.dimensionNum):
            randomStateOut[i] = randomVal[i] * self.regionOperating.size[i] \
             - self.regionOperating.size[i]/2.0 + self.regionOperating.center[i]
        return randomStateOut
        
    def extendTo(self, stateFromIn, stateTowardsIn):
        
        trajectoryOut = Trajectory()
        dists = np.zeros(self.dimensionNum)
        for i in range(self.dimensionNum):
            dists[i] = stateTowardsIn[i] - stateFromIn[i]
            
        distTotal = 0.0
        for i in range(self.dimensionNum):
            distTotal += dists[i]*dists[i]
        distTotal = np.sqrt(distTotal)
        
        incrementTotal = distTotal / DISCREMINZATION_STEP
        # normalize the distance according to the disretization step
        for i in range(self.dimensionNum):
            dists[i] /= incrementTotal
            
        numSegments = int(np.floor(incrementTotal))
        
        stateCurr = np.zeros(self.dimensionNum)
        for i in range(self.dimensionNum):
            stateCurr[i] = stateFromIn[i]
            
        for j in range(numSegments):
            if self.isInCollision(stateCurr):
                return trajectoryOut, False
        
        trajectoryOut.endState = copy.deepcopy(stateTowardsIn)  
        trajectoryOut.totalVariation = distTotal
        
        return trajectoryOut, True
        
        
    def evaluateExtensionCost(self, stateFromIn, stateTowardsIn):

        distTotal = 0.0
        for i in range(self.dimensionNum):
            distTotal += (stateTowardsIn[i] - stateFromIn[i])**2
        return distTotal

        
    def getTrajectory(self, stateFromIn, stateToIn, trajectoryOut):
        stateArr = np.zeros(self.dimensionNum)
        for i in range(self.dimensionNum):
            stateArr[i] = stateToIn[i]
        trajectoryOut.vertices.append(stateArr)
        
    def evaluateCostToGo(self, stateIn):
        radius = 0.0
        for i in range(self.dimensionNum):
            radius += self.regionGoal.size[i]**2
        radius = np.sqrt(radius)
        
        dist = 0.0
        for i in range(self.dimensionNum):
            dist += (stateIn[i] - self.regionGoal.center[i])**2
        dist = np.sqrt(dist)
        
        return dist - radius
        
        
    