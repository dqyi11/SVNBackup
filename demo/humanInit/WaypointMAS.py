import numpy as np

class particle(object):
    
    def __init__(self, parent, dim):
        self.parent = parent
        self.dim = dim
        self.pos = np.zeros(dim)

class WaypointAgent(object):
    
    def __init__(self, index, particleNum):
        self.index = index
        self.particleNum = particleNum
        self.centerPos = []
        self.searchRegionSize = []
        self.particles = []
        self.searchRegion = []
        self.particles = []
        
        
    def setParam(self, center, searchRegionSize):
        self.centerPos = center
        self.searchRegionSize = searchRegionSize
        self.searchRegion = np.zeros((len(center), 2))
        for i in range(len(center)):
            self.searchRegion[i,0] = center[i]-searchRegionSize[i]/2.0
            self.searchRegion[i,1] = center[i]+searchRegionSize[i]/2.0

        self.particles = []
        dim = len(center)
        for i in range(self.particleNum):
            p = particle(self, dim)
            self.particles.append(p)
        
        for d in range(len(self.centerPos)):
            rnd_pos = np.random.uniform(self.searchRegion[d, 0], self.searchRegion[d, 1], self.particleNum)
            for i in range(self.particleNum):
                self.particles[i].pos[d] = rnd_pos[i]
                
        
class WaypointMultiAgentSystem(object):
    
    def __init__(self, particleNum):
        self.agents = []
        self.particleNum = particleNum
        self.maxStepLength = 40.0
        self.fitnessData = []
        #self.worldSize = worldSize
        
        
    def addAgent(self, centerPos, searchRegionSize):
        index = len(self.agents)
        agent = WaypointAgent(index, self.particleNum)
        agent.setParam(centerPos, searchRegionSize)
        self.agents.append(agent)
        
    def evolve(self, stepNum):
        
        for t in range(stepNum):
            self.update()
        
    def update(self):
        
        for i in range(self.particleNum):
    
            agentNum = len(self.agents)
            for t in range(agentNum):
                
                if t==0:
                    deltaVec1 = self.agents[t+1].particles[i].pos - self.agents[t].particles[i].pos
                    deltaVec2 = self.agents[t].particles[i].pos - self.agents[t].particles[i].pos           
                elif t==agentNum-1:
                    deltaVec1 = self.agents[t].particles[i].pos - self.agents[t].particles[i].pos
                    deltaVec2 = self.agents[t-1].particles[i].pos - self.agents[t].particles[i].pos            
                else:
                    deltaVec1 = self.agents[t+1].particles[i].pos - self.agents[t].particles[i].pos
                    deltaVec2 = self.agents[t-1].particles[i].pos - self.agents[t].particles[i].pos
                    
                deltaDist1 = np.sqrt(deltaVec1[0]**2+deltaVec1[1]**2)
                deltaDist2 = np.sqrt(deltaVec2[0]**2+deltaVec2[1]**2)
                
                print str(deltaDist1) + " - " + str(deltaDist2)
                
                normDeltaVec1 = np.zeros(len(deltaVec1))
                normDeltaVec2 = np.zeros(len(deltaVec2))
                if deltaDist1 > 0.0:
                    normDeltaVec1[0] = deltaVec1[0] / deltaDist1
                    normDeltaVec1[1] = deltaVec1[1] / deltaDist1
                if deltaDist2 > 0.0:
                    normDeltaVec2[0] = deltaVec2[0] / deltaDist2
                    normDeltaVec2[1] = deltaVec2[1] / deltaDist2
                
                if deltaDist1 <= self.maxStepLength:
                    deltaDist1 = 0.0
                else:
                    deltaDist1 = deltaDist1 - self.maxStepLength
                    
                if deltaDist2 <= self.maxStepLength:
                    deltaDist2 = 0.0
                else:
                    deltaDist2 = deltaDist2 - self.maxStepLength
                
                
                #print str(deltaDist1) + " - " + str(deltaDist2)  
                newDeltaVec = np.zeros(len(deltaVec1))  
                newDeltaVec[0] = normDeltaVec1[0] * deltaDist1 + normDeltaVec2[0] * deltaDist2
                newDeltaVec[1] = normDeltaVec1[1] * deltaDist1 + normDeltaVec2[1] * deltaDist2
                
                self.agents[t].particles[i].pos[0]  = self.agents[t].particles[i].pos[0] + 0.1 * newDeltaVec[0]
                self.agents[t].particles[i].pos[1]  = self.agents[t].particles[i].pos[1] + 0.1 * newDeltaVec[1]
                
                if self.agents[t].particles[i].pos[0] < self.agents[t].searchRegion[0,0]:
                    self.agents[t].particles[i].pos[0] = self.agents[t].searchRegion[0,0]
                elif self.agents[t].particles[i].pos[0] > self.agents[t].searchRegion[0,1]:
                    self.agents[t].particles[i].pos[0] = self.agents[t].searchRegion[0,1]
                if self.agents[t].particles[i].pos[1] < self.agents[t].searchRegion[1,0]:
                    self.agents[t].particles[i].pos[1] = self.agents[t].searchRegion[1,0]
                elif self.agents[t].particles[i].pos[1] > self.agents[t].searchRegion[1,1]:
                    self.agents[t].particles[i].pos[1] = self.agents[t].searchRegion[1,1]
            
            

                