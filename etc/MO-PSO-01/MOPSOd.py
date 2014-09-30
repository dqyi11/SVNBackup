'''
Created on Sep 30, 2014

@author: daqing_yi
'''

import numpy as np
import copy

class Particle(object):

    def __init__(self, dim):
        self.dim = dim
        self.pos = np.zeros(self.dim, np.float)
        self.vel = np.zeros(self.dim, np.float)
        
        self.pbFitness = None
        self.pbPos = None
        
class Swarm(object):
    
    def __init__(self, particleNum, dim, fitness_dim, fitness_func, reference_fitness, weights):
        self.particleNum = particleNum
        self.dim = dim
        self.fit_dim = fitness_dim
        self.fitness_func = fitness_func
        
        self.gbFitness = None
        self.gbPos = None
        
        self.reference_fitness = reference_fitness
        self.weights = weights
        
        self.particles = []
        for i in range(particleNum):
            self.particles.append(Particle(self.dim))
            
    def initSwarm(self, initRange, chi, phi_p, phi_g):
        self.chi = chi
        self.phi_p = phi_p
        self.phi_g = phi_g
        
        for d in range(self.dim):
            rndInt = np.random.random(self.particleNum) *(initRange[d,1] - initRange[d,0]) + initRange[d,0]
            for i in range(self.particleNum):
                self.particles[i].pos[d] = rndInt[i]
                
        self.updateFitness()
                
    def update(self):
        
        self.updatePos()
        self.updateFitness()
                
    def updatePos(self):
        # update position
        rnd_p = np.random.random(self.particleNum)
        rnd_g = np.random.random(self.particleNum)
        for i in range(self.particleNum):
            p = self.particles[i]
            p.vel = self.chi * (p.vel + self.phi_p * rnd_p[i] *(p.pbPos -p.pos) + self.phi_g * rnd_g[i] * (self.gbPos - p.pos))
            p.pos = p.pos + p.vel    
        
        
    def updateFitness(self):    
        # update fitness
        for p in self.particles:
            f_val = self.fitness_func(p.pos)
            g_val = np.zeros(self.fit_dim, np.float)
            for d in range(self.fit_dim):
                g_val[d] = self.weights[d] * np.abs(f_val[d] - self.reference_fitness[d])
            p.fitness = np.max(g_val)
            
            if p.pbFitness == None or p.pbFitness < p.fitness:
                p.pbFitness = p.fitness
                p.pbPos = copy.deepcopy(p.pos)
            
            if self.gbFitness == None or self.gbFitness < p.fitness:
                self.gbFitness = p.fitness
                self.gbPos = copy.deepcopy(p.pos)
        
        
        
    
        
               