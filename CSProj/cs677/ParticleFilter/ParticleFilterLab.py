import numpy as np 

def Observe(x, y):    
    dA, dB = SensorModel(x,y)
    
    # additive noise
    rA = np.random.normal(dA,1)
    rB = np.random.normal(dB,1)
    
    return rA, rB

def SensorModel(x, y):
    dA = np.sqrt(pow(-100-x,2) + pow(100-y,2))
    dB = np.sqrt(pow(150-x,2) + pow(90-y,2))
    
    return dA, dB  

def LikelihoodWeight(estX, estY, obsRA, obsRB):
    
    estRA, estRB = SensorModel(estX, estY)
    # rA and rB are independent
    rALikelihood = NormalGaussian(estRA, 1, obsRA)
    rBLikelihood = NormalGaussian(estRB, 1, obsRB)
    
    likelihood = rALikelihood * rBLikelihood
    
    return likelihood

def Transit(x, y, theta, d):
    nextX = x + d * np.cos(theta)
    nextY = y + d * np.sin(theta)
    return nextX, nextY

def NormalGaussian(mean, var, x):
    #y = (1/np.sqrt(2 * pi * var)) * np.exp(-math.pow(x-mean, 2) / var
    y = np.exp(- (x-mean)**2 / var)
    return y

def Resample(particles, particuleNum):
    
    newParticles = np.ones((3, particleNum), np.double)
    r = np.random.uniform(0, 1, particleNum)
    C = np.zeros(particleNum)
    C[0] = particles[2,0]
    for i in range(1,particleNum):
        C[i] = C[i-1] + particles[2,i]
    
    for i in range(particleNum):
        j = 0
        while (r[i] > C[j]) and (j < particleNum-1):
            j += 1
        newParticles[0,i] = particles[0,j]
        newParticles[1,i] = particles[1,j]
        newParticles[2,i] = 1./particleNum
    
    return newParticles

def Estimate(particles, particleNum):
    # use average for estimation
    estimatedPos = np.zeros((2))
    variance = np.zeros((2))
    for i in range(particleNum):    
        estimatedPos[0] += particles[2, i] * particles[0, i]
        estimatedPos[1] += particles[2, i] * particles[1, i]
        
    for i in range(particleNum):
        variance[0] += (particles[0, i]-estimatedPos[0])**2
        variance[1] += (particles[1, i]-estimatedPos[1])**2
        
    variance /= particleNum
    return estimatedPos, variance
      
    
def ParticleFilter(particles, observation, particleNum):
    
    newParticles = np.ones((3, particleNum))
    #1. For each particle
    #   sample an estimation from given particles
    d = np.random.normal(5, 1, particleNum)
    theta = np.random.uniform(np.pi/5 - np.pi/36, np.pi/5 + np.pi/36, particleNum)
    for i in range(particleNum):
        newParticles[0, i], newParticles[1, i] = Transit(particles[0, i], particles[1, i], theta[i], d[i])
    
    #2. For each particle
    #   update weight by updating dependent probability
    #   (based on estimated position and observation)
    for i in range(particleNum):
        newParticles[2, i] = LikelihoodWeight(newParticles[0, i], newParticles[1, i], observation[0], observation[1])
    
    #3. Normalize the weights
    newParticles[2,:] /= sum(newParticles[2,:])
    
    #4. Resampling     
    newParticles = Resample(newParticles, particleNum)
    
    return newParticles

if __name__ == '__main__':
    
    # simulation step number
    step = 20
    # particle number 
    particleNum = 100
    
    loadDataInput = True

    # pos[0,:] = X, pos[1,:] = Y, pos[2,:] = theta
    pos = np.zeros((3, step+1))
    # estPos[0,:] = estX, estPos[1,:] = estY
    estPos = np.zeros((2, step+1))
    # estPos[0,:] = varX, estPos[1,:] = varY
    variance = np.zeros((2, step+1))

    # obs[0,:] = rA, obs[1,:] = rB
    obs = np.zeros((2, step+1))
    
    if loadDataInput == True:
        # load positions and observations from file
        f = open('input.txt')
        t = 1
        for line in f.readlines():
            file = line.replace('\\', '')
            output = np.double(file.split('&'))
            pos[0, t] = output[0]
            pos[1, t] = output[1]
            obs[0, t] = output[2]
            obs[1, t] = output[3]   
            t += 1     
        f.close()        
    else:
        # randomly generate positions and observations
        # init position
        pos[0, 0] = np.random.normal(0, 1)
        pos[1, 0] = np.random.normal(0, 1)
    
        pos[2, :] = np.random.uniform(np.pi/5-np.pi/36, np.pi/5+np.pi/36, step+1)
        d = np.random.normal(5, 1, step+1)
        obs[0, 0], obs[1, 0] = Observe(pos[0, 0], pos[1, 0])
    
        # generate motion trajectory
        for t in range(1, step+1):
            pos[0, t], pos[1, t] = Transit(pos[0, t-1],pos[1, t-1],pos[2, t-1],d[t-1])
            # get observation
            obs[0, t], obs[1, t] = Observe(pos[0, t], pos[1, t])
        
    # particlesSeq[0,:,:] = X, particlesSeq[1,:,:] = Y, particlesSeq[2,:,:] = weight 
    particlesSeq = np.zeros((step+1, 3, particleNum))
    
    # init particles
    particlesSeq[0, 0] = np.random.normal(0, 1, particleNum)
    particlesSeq[0, 1] = np.random.normal(0, 1, particleNum)
    particlesSeq[0, 2] = np.ones(particleNum) * 1/particleNum
    
    # estimate init position
    estPos[:, 0], variance[:, 0] = Estimate(particlesSeq[0], particleNum)
        
    for t in range(1, step+1):
        # run particle filter
        particlesSeq[t] = ParticleFilter(particlesSeq[t-1], obs[:, t], particleNum)
        # Estimation
        estPos[:, t], variance[:, t] = Estimate(particlesSeq[t], particleNum)
        
    # print 
    for t in range(1, step+1):
        print ("{} & {} & {} & {} \\\\".format(pos[0, t], pos[1, t], obs[0, t], obs[1, t]))
    
    print (" ")  
    for t in range(1, step+1):
        print ("{} & {} & {} & {} \\\\".format(estPos[0, t], estPos[1, t], variance[0, t], variance[1, t]))
        
    