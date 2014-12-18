'''
Created on 2013-5-13

@author: Walter
'''
 
    
import numpy as np
import matplotlib.pyplot as plt 

def Observe(x, y):    
    dA, dB = SensorModel(x,y)
    
    rA = np.random.normal(dA,1)
    rB = np.random.normal(dB,1)
    
    return rA, rB

def SensorModel(x, y):
    dA = np.sqrt((-100-x)**2 + (100-y)**2)
    dB = np.sqrt((150-x)**2 + (90-y)**2)
    
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
    y = np.exp(-(x-mean)**2 / var)
    return y

def Resample(particles, particuleNum):
    
    newParticles = np.ones((3, particleNum), np.double)
    r = np.random.uniform(0, 1, particleNum)
    C = np.zeros(particleNum, np.double)
    C[0] = particles[2,0]
    for i in range(1,particleNum):
        C[i] = C[i-1] + particles[2,i]
        #print "C[{}] = {}".format(i, C[i])
    
    
    for i in range(particleNum):
        j = 0
        while (r[i] > C[j]) and (j < particleNum-1):
            j += 1
        #print "i is {}. r is {}, C[{}] is {}".format(i, r[i], j, C[j])
        newParticles[0,i] = particles[0,j]
        newParticles[1,i] = particles[1,j]
        newParticles[2,i] = 1./particleNum
    
    return newParticles

def Estimate(particles, particleNum):
    # use average for estimation
    estimatedPos = np.zeros((2), np.double)
    variance = np.zeros((2), np.double)
    for i in range(particleNum):    
        estimatedPos[0] += particles[2, i] * particles[0, i]
        estimatedPos[1] += particles[2, i] * particles[1, i]
        
    for i in range(particleNum):
        variance[0] += (particles[0, i]-estimatedPos[0])**2
        variance[1] += (particles[1, i]-estimatedPos[1])**2
        
    variance[0] /= particleNum
    variance[1] /= particleNum
    return estimatedPos, variance
      
    
def ParticleFilter(particles, observation, particleNum):
    
    newParticles = np.ones((3, particleNum), np.double)
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
    
    #4. Resmapling     
    newParticles = Resample(newParticles, particleNum)
    
    return newParticles

if __name__ == '__main__':
    
    # simulation step number
    step = 20
    # particle number 
    particleNum = 100
    
    loadInput = True

    # Pos[0,:] = X
    # Pos[1,:] = Y
    # Pos[2,:] = theta
    pos = np.zeros((3, step+1), np.double)
    estPos = np.zeros((2, step+1), np.double)
    variance = np.zeros((2, step+1), np.double)
    
    estErr = np.zeros((2, step+1), np.double)
    
    # Obs[0,:] = rA
    # Obs[1,:] = rB
    obs = np.zeros((2, step+1), np.double)
    
    t = 1
    if loadInput == True:
        f = open('input.txt')
        for line in f.readlines():
            line = line.replace('\\', '')
            output = line.split('&')
            pos[0, t] = float(output[0])
            pos[1, t] = float(output[1])
            obs[0, t] = float(output[2])
            obs[1, t] = float(output[3]) 
            t += 1     
        f.close()        
    else:      
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
        
    # ParticlesSeq[0,:,:] = X
    # ParticlesSeq[1,:,:] = Y
    # ParticlesSeq[2,:,:] = weight 
    #particlesSeq = [im for im in zeros((step,3,particleNum),double)]
    particlesSeq = np.zeros((step+1, 3, particleNum), np.double)
    
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
        
    for t in range(step+1):
        estErr[0, t] = estPos[0, t] - pos[0, t]
        estErr[1, t] = estPos[1, t] - pos[1, t]
        
    # print 
    #for t in range(1, step+1):
    #    print "{} & {} & {} & {} \\\\".format(pos[0, t], pos[1, t], obs[0, t], obs[1, t])
    
    #print " "  
    #for t in range(1, step+1):
    #    print "{} & {} & {} & {} \\\\".format(estPos[0, t], estPos[1, t], variance[0, t], variance[1, t])
        
    # plot
    fig = plt.figure()
    ax = fig.add_subplot(111)

    ax.plot(pos[0, :], pos[1, :], '-o')
    ax.plot(estPos[0,:], estPos[1,:])   

    plt.show()
    