import copy
import numpy as np

class Agent(object):
    
    def __init__(self):
        self.observeRange = 1
        self.discountFactor = 0.3
        self.pos = None
        
    def getObservation(self, pos, hexamap, rewardDistribution):
        reward = 0.0
        if self.observeRange==0:
            reward = rewardDistribution[pos[0],pos[1]]
        else:
            hexes = hexamap.getHexesByRadius(pos, self.observeRange)
            for hx in hexes:
                if hx[0]==pos[0] and hx[1]==pos[1]:
                    reward += rewardDistribution[hx[0],hx[1]]
                else:
                    reward += self.discountFactor*rewardDistribution[hx[0],hx[1]]
        return reward
        
    def applyObservation(self, pos, hexamap, rewardDistribution):
        hexes = hexamap.getHexesByRadius(pos, self.observeRange)
        for hx in hexes:
            if hx[0]==pos[0] and hx[1]==pos[1]:
                rewardDistribution[hx[0],hx[1]] = 0.0
            else:
                rewardDistribution[hx[0],hx[1]] = rewardDistribution[hx[0],hx[1]] * (1 - self.discountFactor)
        return rewardDistribution
    
    def getPathReward(self, path, hexamap, rewardDistribution):
        rewardDist = copy.deepcopy(rewardDistribution)
        pathReward = 0.0
        for pos in path:
            pathReward += self.getObservation(pos, hexamap, rewardDist)
            rewardDist = self.applyObservation(pos, hexamap, rewardDist)

        return pathReward
    
    def getPathRewardVec(self, path, hexamap, rewardDistributions):
        rewardDist = copy.deepcopy(rewardDistributions)
        dim = len(rewardDistributions)
        pathReward = np.zeros(dim)
        for pos in path:
            for d in range(dim):
                pathReward[d] += self.getObservation(pos, hexamap, rewardDist[d])

        return pathReward
    
    def getObservationVec(self, pos, hexamap, rewardDistributions):
        dim = len(rewardDistributions)
        reward = np.zeros(dim)
        for d in range(dim):
            reward[d] = self.getObservation(pos, hexamap, rewardDistributions[d])
        return reward
        