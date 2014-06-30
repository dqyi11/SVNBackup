'''
Created on 2013-11-23

@author: Walter
'''

import numpy as np;
from Particle import *;
import matplotlib.pyplot as plt;
#import pygame as pg;

class Swarm(object):

    def __init__(self, particleNum, particleDimension):

        self.particleNum = particleNum;
        self.particleDimension = particleDimension;
        
        self.particles = [];
        self.dominatedParticles = [];
        self.nondominatedParticles = [];
        for i in range(self.particleNum):
            particle = Particle(i, particleDimension);
            self.particles.append(particle);
            self.dominatedParticles.append(particle);
            
        
            
        self.histCentroid = [];
        self.histAvgFitness = [];
        self.histCentroidMaximin = [];
        self.histGlobalbestPos = [];
        self.histGlobalBestmaximin = [];
        
        self.dominatedSet = [];
        self.nondominatedSet = [];
        
        self.showCentroidHist = False;
        self.showAverageFitness = False;
        self.showMaximinOfCentroid = False;
        self.showGlobalBestPosition = False;
        
        self.maximinOnNondominatedOnly = False;
        
    
    def setParam(self, phi1, phi2, inertia, objfuncs):
        
        self.phi1 = phi1;
        self.phi2 = phi2;
        self.inertia = inertia;
        self.objfuncs = [];
        self.objnum = len(objfuncs);
        
        for func in objfuncs:
            self.objfuncs.append(func);
            
        self.swarm_centroid = [];
        self.average_fitness = [];
        self.swarm_centroid_fitness = [];
        
            
    def setDisplayParam(self, width, height, scale, interval = 0.01):
        
        self.width = width;
        self.height = height;
        self.scale = scale;
        self.interval = interval;
        
        
    def initParticles(self, worldrange):
        
        self.worldrange = worldrange;
        for p in self.particles:
            for d in range(self.particleDimension):
                p.pos[0,d] = (np.random.random() - 0.5) * worldrange[d];
                p.vel[0,d] = 0.0;
            
        self.firstRun = True;
        self.globalbestAgentIdx = 0;
        
        self.swarm_centroid = np.zeros((1,self.particleDimension), np.float);
        self.swarm_centroid_fitness = np.zeros((1,self.objnum), np.float);
        self.average_fitness = np.zeros((1, self.objnum), np.float); 
        for p in self.particles:
            x_pos = [];
            for d in range(self.particleDimension):
                self.swarm_centroid[0,d] += p.pos[0,d];
                x_pos.append(p.pos[0,d]);
            for k in range(self.objnum):
                self.average_fitness[0,k] = self.objfuncs[k](x_pos);
            
        self.swarm_centroid = self.swarm_centroid / self.particleNum;
        self.average_fitness = self.average_fitness / self.particleNum;
        cen_pos = [];
        for d in range(self.particleDimension):
            cen_pos.append(self.swarm_centroid[0,d]);
        for k in range(self.objnum):
            self.swarm_centroid_fitness[0,k] = self.objfuncs[k](cen_pos);
            
    def categorizeRefSet(self):
    
        # find dominated or nondominated
        for a in self.referenceSet:
            for b in self.referenceSet:
                if b.index == a.index:
                    continue;
                # assume a is dominated by b
                a_dominated_by_b = True;
                for k in range(self.objnum):
                    if self.objfuncs[k](b.pos) > self.objfuncs[k](a.pos):
                        # as long as we find one case that a is smaller,
                        # a is not dominated by b
                        a_dominated_by_b = False;
                        
                # if a is dominated by any other, a is not nondominated
                if a_dominated_by_b==True:
                    a.nondominated = False;
            if a.nondominated == True:
                self.nondominatedSet.append(a);
            else:
                self.dominatedSet.append(a);
                
                    
    def update(self):
        
        if self.firstRun == True:
            self.firstRun = False;
            
        for p in self.particles:
            U1 = np.matrix(np.zeros((self.particleDimension, self.particleDimension), np.float));
            U2 = np.matrix(np.zeros((self.particleDimension, self.particleDimension), np.float));        
            for d in range(self.particleDimension):
                U1[d,d] = np.random.random();
                U2[d,d] = np.random.random();
                
            localForce = self.phi1 * U1 * (p.localbestPos.T - p.pos.T);
            globalForce = self.phi2 * U2 * (self.particles[self.globalbestAgentIdx].localbestPos.T - p.pos.T);
            p.vel = self.inertia * (p.vel + localForce.T + globalForce.T);
            p.pos = p.pos + p.vel;
            
            for d in range(self.particleDimension):
                if p.pos[0,d] > self.worldrange[d] / 2:
                    p.pos[0,d] = self.worldrange[d] / 2;
                elif p.pos[0,d] < - self.worldrange[d] / 2:
                    p.pos[0,d] = - self.worldrange[d] / 2;
                    
        self.dominatedParticles = [];
        self.nondominatedParticles = [];        
        
        self.average_fitness = np.zeros((1, self.objnum), np.float);
        for i in range(self.particleNum):
            p = self.particles[i];
            x_i = [];
            
            for t in range(self.particleDimension):
                x_i.append(p.pos[0,t]);
            for k in range(self.objnum):
                self.average_fitness[0, k] += self.objfuncs[k](x_i);
            
            p.nondominated = True;
            minObj = [];
            
            for j in range(self.particleNum):
                if j==i:
                    continue;
                q = self.particles[j];
                
                j_dominate_i = True;
                x_j = [];                
                for d in range(self.particleDimension):
                    x_j.append(q.pos[0,d]);
                
                for k in range(self.objnum):
                    delta_data = self.objfuncs[k](x_i) - self.objfuncs[k](x_j);
                    if delta_data <= 0.0:
                        j_dominate_i = False;                
                if j_dominate_i == True:
                    p.nondominated = False;
                    
            if p.nondominated == True:
                self.nondominatedParticles.append(p);
            else:
                self.dominatedParticles.append(p);
                
                
        for i in range(self.particleNum):
            p = self.particles[i];
            x_i = [];
            
            for t in range(self.particleDimension):
                x_i.append(p.pos[0,t]);            
            if self.maximinOnNondominatedOnly == False:
                for j in range(self.particleNum):
                    if j == i:
                        continue;
                    q = self.particles[j];
                    
                    fit_delta = [];
                    x_j = [];                
                    for d in range(self.particleDimension):
                        x_j.append(q.pos[0,d]);
                    
                    for k in range(self.objnum):
                        delta_data = self.objfuncs[k](x_i) - self.objfuncs[k](x_j);
                        fit_delta.append(delta_data);
                    minObj.append(np.amin(fit_delta));
            else:
                for q in self.nondominatedParticles:
                    if q.index == p.index:
                        continue;
                    fit_delta = [];
                    x_j = [];                
                    for d in range(self.particleDimension):
                        x_j.append(q.pos[0,d]);
                    
                    for k in range(self.objnum):
                        delta_data = self.objfuncs[k](x_i) - self.objfuncs[k](x_j);
                        fit_delta.append(delta_data);
                    minObj.append(np.amin(fit_delta));
            
            p.fitness = np.amax(minObj);
            
            if self.firstRun == True:
                p.localbestFitness = p.fitness;
                p.localbestPos = p.pos;
            
            if p.fitness < p.localbestFitness:
                p.localbestFiness = p.fitness;
                p.localbestPos = p.pos;
        
            if p.fitness < self.particles[self.globalbestAgentIdx].fitness:
                self.globalbestAgentIdx = i;       
                    
        self.swarm_centroid = np.zeros((1,self.particleDimension), np.float);
        self.swarm_centroid_fitness = np.zeros((1,self.objnum), np.float);
        for p in self.particles:
            for d in range(self.particleDimension):
                self.swarm_centroid[0,d] += p.pos[0,d];
                
        self.swarm_centroid = self.swarm_centroid / self.particleNum;
        self.average_fitness = self.average_fitness / self.particleNum;

        cen_pos = [];
        for d in range(self.particleDimension):
            cen_pos.append(self.swarm_centroid[0,d]);
        for k in range(self.objnum):
            self.swarm_centroid_fitness[0,k] = self.objfuncs[k](cen_pos);
            
        self.histCentroid.append(self.swarm_centroid);
        self.histAvgFitness.append(self.average_fitness);
        self.histCentroidMaximin.append(self.calculateMaximin(cen_pos));
        
        globalBestP = self.particles[self.globalbestAgentIdx];
        globalbest_pos = [];
        for d in range(self.particleDimension):
            globalbest_pos.append(globalBestP.pos[0,d]);
        self.histGlobalbestPos.append(globalbest_pos);
        self.histGlobalBestmaximin.append(self.calculateMaximin(globalbest_pos));

        
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
