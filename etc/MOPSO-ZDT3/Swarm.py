'''
Created on 2013-11-23

@author: Walter
'''

import numpy as np;
from Particle import *;
import matplotlib.pyplot as plt;
import pickle;
from PerformanceLogger import *;
from GlobalBestSet import *;

class Swarm(object):

    def __init__(self, particleNum, particleDimension):

        self.particleNum = particleNum;
        self.particleDimension = particleDimension;
        
        self.particles = [];
        self.globalbestPos = np.matrix(np.zeros((1,self.particleDimension), np.float));
        self.globalbestFitness = 0.0;
        self.globalbestFitnessPos = np.matrix(np.zeros((1,self.particleDimension), np.float));
        
        self.dominatedParticles = [];
        self.nondominatedParticles = [];
        for i in range(self.particleNum):
            particle = Particle(i, particleDimension);
            self.particles.append(particle);
            self.dominatedParticles.append(particle);
        
        self.logger = PerformanceLogger(self);
        self.globalbestSet = GlobalBestSet(self.particleDimension);
        
        self.paretoX = None;
        self.paretoY = None;
        self.localParetoX = None;
        self.localParetoY = None;
    
    def setParam(self, phi1, phi2, inertia, objfuncs):
        
        self.phi1 = phi1;
        self.phi2 = phi2;
        self.inertia = inertia;
        self.objfuncs = [];
        self.objnum = len(objfuncs);
        
        for func in objfuncs:
            self.objfuncs.append(func);
            
        for p in self.particles:
            p.evaluationPos = np.matrix(np.zeros((1,self.objnum), np.float));
            
            
    def setDisplayParam(self, width, height, scale, interval = 0.01):
        
        self.width = width;
        self.height = height;
        self.scale = scale;
        self.interval = interval;
        
        
    def initParticles(self, worldrange):
        
        self.worldrange = worldrange;
        for p in self.particles:
            for d in range(self.particleDimension):
                p.pos[0,d] = np.random.random() * (worldrange[d][1] - worldrange[d][0]) + worldrange[d][0];
                p.vel[0,d] = 0.0;
            
        self.firstRun = True;
            
    def calcObjFunc(self, pos, k):
        tPos = [];                
        for d in range(self.particleDimension):
            tPos.append(pos[0,d]);
        
        val = self.objfuncs[k](tPos);
        return val;
        
                    
    def update(self):
        
        if self.firstRun == True:
            self.firstRun = False;
            self.globalbestFitnessPos = self.particles[0].getCurrentPos();
        
        # apply velocity    
        for p in self.particles:

            for d in range(self.particleDimension):
                u1 = np.random.random();
                u2 = np.random.random();
                localForce = self.phi1 * u1 * (p.localbestPos[0,d] - p.pos[0,d]);
                
                
                if p.nondominated == True:
                    globalForce = self.phi2 * u2 * (p.globalbestPos[0,d] - p.pos[0,d]);
                else:
                    globalForce = self.phi2 * u2 * (self.globalbestPos[0,d] - p.pos[0,d]);
                
                #globalForce = self.phi2 * u2 * (self.globalbestPos[0,d] - p.pos[0,d]);
                
                p.vel[0,d] = self.inertia * (p.vel[0,d] + localForce + globalForce)
                p.pos[0,d] = p.pos[0,d] + p.vel[0,d];
                                
                if p.pos[0,d] > self.worldrange[d][1]:
                    p.pos[0,d] = self.worldrange[d][1];
                elif p.pos[0,d] < self.worldrange[d][0]:
                    p.pos[0,d] = self.worldrange[d][0];

        # determine the property of dominance
        self.dominatedParticles = [];
        self.nondominatedParticles = [];  
        
        for p in self.particles:
            for k in range(self.objnum):
                p.evaluationPos[0,k] = self.calcObjFunc(p.pos, k);
               
        for i in range(self.particleNum):
            p = self.particles[i];
           
            p.nondominated = True;
            minObj = [];
            
            for j in range(self.particleNum):
                if j==i:
                    continue;
                q = self.particles[j];
                
                fit_delta = [];                
                for k in range(self.objnum):
                    delta_data = p.evaluationPos[0,k] - q.evaluationPos[0,k]
                    fit_delta.append(delta_data);
                minObj.append(np.amin(fit_delta));
                
            if np.amax(minObj) <= 0:
                p.nondominated = True;
            else:
                p.nondominated = False;
                    
            if p.nondominated == True:
                self.nondominatedParticles.append(p);
                self.globalbestSet.addObs(p.getCurrentPos(), True);
                p.personalbestSet.addObs(p.getCurrentPos(), True);
            else:
                self.dominatedParticles.append(p);
        
        # generate new fitness
        for i in range(self.particleNum):
            p = self.particles[i];
            
            minObj = [];        

            for q in self.particles:
                if q.index == p.index:
                    continue;
                fit_delta = [];
                
                for k in range(self.objnum):
                    delta_data = p.evaluationPos[0,k] - q.evaluationPos[0,k];
                    fit_delta.append(delta_data);
                minObj.append(np.amin(fit_delta));
            
            p.fitness = np.amax(minObj);
            
            if self.firstRun == True:
                p.localbestFitness = p.fitness;
            
            if p.fitness < p.localbestFitness:
                p.localbestFitness = p.fitness
                p.localbestFitnessPos = p.getCurrentPos();
                
            p.localbestPos = p.localbestFitnessPos;
        
            if p.fitness < self.globalbestFitness:
                self.globalbestFitness = p.fitness;
                self.globalbestFitnessPos = p.getCurrentPos();
            
            if p.nondominated == True:
                #p.globalbestPos = p.getCurrentPos();
                p.globalbestPos = self.globalbestSet.sample();
                p.personalbestPos = p.personalbestSet.sample();
        
        self.globalbestPos = self.globalbestFitnessPos;     
        self.globalbestSet.clearObs(); 
                                  
        
    def calculateMaximin(self, pos):
        
        minObj = [];
        for p in self.particles:
            x_i = [];            
            for t in range(self.particleDimension):
                x_i.append(p.pos[0,t]);
                            
            fit_delta = [];
            for k in range(self.objnum):
                fit_delta.append(self.objfuncs[k](pos) - self.objfuncs[k](x_i));
               
            minObj.append(np.amin(fit_delta));
            
        return np.amax(minObj);
    
    def calcFitMinAbsDelta(self, pos1, pos2):
        fit_abs_delta = [];
        for k in range(self.objnum):
            fit_abs_delta.append(np.abs(self.calcObjFunc(pos1, k) - self.calcObjFunc(pos2, k)));
        return np.amin(fit_abs_delta);
    
    def calcFitAbsDelta(self, pos1, pos2):
        delta = 0.0;
        for k in range(self.objnum):
            delta += (self.calcObjFunc(pos1, k) - self.calcObjFunc(pos2, k))**2;
        return np.sqrt(delta);
        

    def getRandomPos(self):
        pos = np.matrix(np.zeros((1,self.particleDimension), np.float));
        for d in range(self.particleDimension):
            pos[0,d] = np.random.random() * (self.worldrange[d][1] - self.worldrange[d][0]) + self.worldrange[d][0];
        return pos;
                
            