'''
Created on Dec 18, 2014

@author: hcmi
'''

import numpy as np

def NormalGaussian(mean, var, x):
    #y = (1/np.sqrt(2 * pi * var)) * np.exp(-math.pow(x-mean, 2) / var
    y = np.exp(-(x-mean)**2 / var)
    return y

class ParticleFilter(object):

    def __init__(self, particleNum, stateDim, obsDim, mModel, oModel):
        self.particleNum = particleNum
        self.mModel = mModel
        self.oModel = oModel
        self.stateDim = stateDim
        self.obsDim = obsDim
        
        self.particles = np.zeros((self.particleNum, self.stateDim+1), np.float)
        randVal = np.random.random(self.particleNum*self.stateDim)
        
        for i in range(self.particleNum):
            for d in range(self.stateDim):
                self.particles[i,d] = randVal[d + i*self.stateDim]
            self.particles[i, self.stateDim] = 1.0/self.stateDim
            
    def update(self, observation, sysinput):
        
        newParticles = np.ones((self.particleNum, self.stateDim+1), np.double)
        #1. For each particle
        #   sample an estimation from given particles
        #d = np.random.normal(5, 1, self.particleNum)

        for i in range(self.particleNum):
            newParticles[i, 0], newParticles[i, 1] = self.mModel(self.particles[i, 0:self.stateDim], sysinput)
    
        #2. For each particle
        #   update weight by updating dependent probability
        #   (based on estimated position and observation)
        for i in range(self.particleNum):
            newParticles[i, self.stateDim] = self.likelihoodWeight(self.particles[i, 0:self.stateDim], observation)
    
        #3. Normalize the weights
        newParticles[:,self.stateDim] /= sum(newParticles[:, self.stateDim])
        
        #4. Resmapling     
        newParticles = self.resample(newParticles)
    
        self.particles = newParticles
        
    def resample(self, particles):
    
        newParticles = np.ones((self.particleNum, self.stateDim+1), np.double)
        r = np.random.uniform(0, 1, self.particleNum)
        C = np.zeros(self.particleNum, np.double)
        C[0] = particles[0, self.stateDim]
        for i in range(1, self.particleNum):
            C[i] = C[i-1] + particles[i, self.stateDim]
            #print "C[{}] = {}".format(i, C[i])
        
        for i in range(self.particleNum):
            j = 0
            while (r[i] > C[j]) and (j < self.particleNum-1):
                j += 1
            #print "i is {}. r is {}, C[{}] is {}".format(i, r[i], j, C[j])
            for d in range(self.stateDim):
                newParticles[i, d] = particles[j, d]
            #newParticles[i, 0] = particles[j, 0]
            #newParticles[i, 1] = particles[j, 1]
            newParticles[i, self.stateDim] = 1./self.particleNum
        
        return newParticles
    
    def likelihoodWeight(self, estPos, obs):
    
        estObs = self.oModel(estPos)
        
        # rA and rB are independent
        likelihoods = np.ones(self.obsDim)
        for o in range(self.obsDim):
            likelihoods[o] = NormalGaussian(estObs[o], 1, obs[o])
        
        likelihood = 1.0
        for o in range(self.obsDim):
            likelihood *= likelihoods[o]       
        return likelihood
    
    def estimate(self, particles):
        # use average for estimation
        estimatedPos = np.zeros((self.stateDim), np.double)
        variance = np.zeros((self.stateDim), np.double)
        for i in range(self.particleNum):
            for d in range(self.stateDim):
                estimatedPos[d] += particles[i, self.stateDim]*particles[i,d]
            
        
        for d in range(self.stateDim):
            for i in range(self.particleNum):
                variance[d] += (particles[i, d]-estimatedPos[d])**2
            variance[d] /= self.particleNum
            
        return estimatedPos, variance
        
        
    
