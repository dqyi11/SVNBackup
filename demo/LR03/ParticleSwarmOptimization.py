import numpy as np
import copy

class Particle(object):
    
    def __init__(self, dim):
        self.pos = np.zeros(dim)
        self.fitness = np.inf
        self.pb = np.zeros(dim)
        self.pbFitness = np.inf
        self.vel = np.zeros(dim)
        

class Swarm(object):


    def __init__(self, particle_num, dim, posRange, fitnessFunc, chi, phi_p, phi_g):
        self.particle_num = particle_num
        self.dim = dim        
        self.fitnessFunc = fitnessFunc
        self.posRange = posRange
        self.chi = chi
        self.phi_p = phi_p
        self.phi_g = phi_g
        
        self.itrCnt = 0
        self.gb = np.zeros(dim)
        self.gbFitness = np.inf
        self.particles = []
        for i in range(self.particle_num):
            p = Particle(self.dim)
            p.pos = np.random.random(self.dim) * (posRange[1]-posRange[0]) + posRange[0]
            p.pb = copy.deepcopy(p.pos)
            self.particles.append(p)
            
            
    def next(self):
        
        self.itrCnt += 1
        
        for p in self.particles:
            p.fitness = self.fitnessFunc(p.pos)
            if p.fitness < p.pbFitness:
                p.pb = copy.deepcopy(p.pos)
                p.pbFitness = p.fitness
                
            if p.fitness < self.gbFitness:
                self.gb = copy.deepcopy(p.pos)
                self.gbFitness = p.fitness
                
        for p in self.particles:
            for d in range(self.dim):
                p.vel[d] = self.chi * ( p.vel[d] + self.phi_p * (p.pb[d] - p.pos[d]) + self.phi_g * (self.gb[d] - p.pos[d]) )
                p.pos[d] = p.pos[d] + p.vel[d]
                
            



        