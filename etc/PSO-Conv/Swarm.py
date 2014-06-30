'''
Created on 2013-11-23

@author: Walter
'''

import numpy as np;
from Particle import *;
import matplotlib.pyplot as plt;
import pickle;

class Swarm(object):

    def __init__(self, particleNum, particleDimension):

        self.particleNum = particleNum
        self.particleDimension = particleDimension
        
        self.particles = []
        self.globalbestPos = np.matrix(np.zeros((1,self.particleDimension), np.float))
        self.globalbestFitness = 100.0
        self.globalbestFitnessPos = np.matrix(np.zeros((1,self.particleDimension), np.float))
        
        for i in range(self.particleNum):
            particle = Particle(i, particleDimension)
            self.particles.append(particle)
    
    def setParam(self, phi1, phi2, inertia, objfunc):
        
        self.phi1 = phi1
        self.phi2 = phi2
        self.inertia = inertia
        self.objfunc = objfunc
            
    def initParticles(self, worldrange):
        
        self.worldrange = worldrange
        for p in self.particles:
            for d in range(self.particleDimension):
                p.pos[0,d] = np.random.random() * (worldrange[d][1] - worldrange[d][0]) + worldrange[d][0]
                p.vel[0,d] = 0.0
                
        self.firstRun = True
                    
    def update(self):
        
        if self.firstRun == True:
            self.firstRun = False
            self.globalbestFitnessPos = self.particles[0].getCurrentPos()
        
        norms = []
        # apply velocity    
        for p in self.particles:

            for d in range(self.particleDimension):
                u1 = np.random.random()
                u2 = np.random.random()
                localForce = self.phi1 * u1 * (p.localbestPos[0,d] - p.pos[0,d])
                globalForce = self.phi2 * u2 * (self.globalbestPos[0,d] - p.pos[0,d])
                
                p.vel[0,d] = self.inertia * (p.vel[0,d] + localForce + globalForce)
                p.pos[0,d] = p.pos[0,d] + p.vel[0,d]
                                
                if p.pos[0,d] > self.worldrange[d][1]:
                    p.pos[0,d] = self.worldrange[d][1]
                elif p.pos[0,d] < self.worldrange[d][0]:
                    p.pos[0,d] = self.worldrange[d][0]
                    
                norms.append(np.linalg.norm(p.pos))
                
        self.bound = np.max(norms)

        # determine the property of dominance    
        for p in self.particles:
            p.fitness = self.objfunc(p.pos)
            
            if p.fitness < p.localbestFitness:
                p.localbestPos = p.getCurrentPos()
                p.localbestFitness = p.fitness
                
            if p.fitness < self.globalbestFitness:
                self.globalbestFitnessPos = p.getCurrentPos()
                self.globalbestFitness = p.fitness
            
