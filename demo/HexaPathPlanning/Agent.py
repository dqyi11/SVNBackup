class Agent(object):
    
    def __init__(self):
        self.observeRange = 1
        self.discountFactor = 0.3
        self.pos = None
        
    def getObservation(self, pos, hexamap, rewardDistribution):
        reward = 0.0
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
        