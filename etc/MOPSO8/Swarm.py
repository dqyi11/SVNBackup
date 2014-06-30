'''
Created on 2013-11-23

@author: Walter
'''

import numpy as np;
from Particle import *;
import matplotlib.pyplot as plt;
#import pygame as pg;
import pickle;
from GlobalBestSet import *;
from PerformanceLogger import *;

class Swarm(object):

    def __init__(self, particleNum, particleDimension):

        self.particleNum = particleNum;
        self.particleDimension = particleDimension;
        
        self.particles = [];
        self.globalbestPos = np.matrix((1, self.particleDimension), np.float);
        self.globalbestFitness = 0.0;
        self.globalbestFitnessPos = self.globalbestPos;
        
        self.dominatedParticles = [];
        self.nondominatedParticles = [];
        for i in range(self.particleNum):
            particle = Particle(i, particleDimension);
            self.particles.append(particle);
            self.dominatedParticles.append(particle);

        
        self.dominatedSet = [];
        self.nondominatedSet = [];
        
        self.gbSet = GlobalBestSet(self.particleDimension);
        self.logger = PerformanceLogger(self);

        
        #self.gbSet = PersonalBestSet(0, self.particleDimension);
    
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
            
    def categorizeRefSet(self, loadFromFile=False, nondomSetFile=None, domSetFile=None):
    
        if loadFromFile == False:
            # find dominated or nondominated
            for a in self.referenceSet:
                
                #maxObj = [];
                minObj = [];
                for b in self.referenceSet:
                    if b.index == a.index:
                        continue;
                    fit_delta = [];
                    
                    for k in range(self.objnum):
                        a_k_obj = self.calcObjFunc(a.pos, k);
                        b_k_obj = self.calcObjFunc(b.pos, k);
                        fit_delta.append(a_k_obj - b_k_obj);
                        
                    #maxObj.append(np.amax(fit_delta));
                    minObj.append(np.amin(fit_delta));

                if np.amax(minObj) <= 0:
                    self.nondominatedSet.append(a);
                else:
                    self.dominatedSet.append(a);
        else:
            self.loadNondominatedSet(nondomSetFile);
            self.loadDominatedSet(domSetFile);
                
        
        self.paretoVar = [];
        for d in range(self.particleDimension):
            dPos = [];
            for s in self.nondominatedSet:
                dPos.append(s.pos[0,d]);
            self.paretoVar.append(np.var(dPos));
            
    def dumpNondominatedSet(self, outFile):
        pickle.dump(self.nondominatedSet, open(outFile, 'wb'));
    def dumpDominatedSet(self, outFile):
        pickle.dump(self.dominatedSet, open(outFile, 'wb'));
    def loadNondominatedSet(self, inFile):
        self.nondominatedSet = pickle.load(open(inFile, 'rb'));
    def loadDominatedSet(self, inFile):
        self.dominatedSet = pickle.load(open(inFile, 'rb'));
        
                
    def getPosVariance(self):
        
        posVariance = [];
        for d in range(self.particleDimension):
            dimPos = [];
            for s in self.particles:
                dimPos.append(s.pos[0,d]);
            posVariance.append(np.var(dimPos));
        return posVariance;
    
    def getFitVariance(self):
        
        particleFit = [];
        
        for s in self.particles:
            particleFit.append(s.fitness);
        return np.var(particleFit);
    
    def calcObjFunc(self, pos, k):
        tPos = [];                
        for d in range(self.particleDimension):
            tPos.append(pos[0,d]);
        
        val = self.objfuncs[k](tPos);
        return val;
        
                    
    def update(self):
        
        if self.firstRun == True:
            self.firstRun = False;
            self.globalbestPos = self.particles[0].pos;
        
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
                p.vel[0,d] = self.inertia * (p.vel[0,d] + localForce + globalForce)
                p.pos[0,d] = p.pos[0,d] + p.vel[0,d];
                
                if p.pos[0,d] > self.worldrange[d] / 2:
                    p.pos[0,d] = self.worldrange[d] / 2;
                elif p.pos[0,d] < - self.worldrange[d] / 2:
                    p.pos[0,d] = - self.worldrange[d] / 2;
                    

        # determine the property of dominance
        self.dominatedParticles = [];
        self.nondominatedParticles = []; 
        
        self.gbSet.clearObs();
         
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
                    delta_data = self.calcObjFunc(p.pos, k) - self.calcObjFunc(q.pos, k);
                    fit_delta.append(delta_data);
                minObj.append(np.amin(fit_delta));
                
            if np.amax(minObj) <= 0:
                p.nondominated = True;
            else:
                p.nondominated = False;
                
                    
            if p.nondominated == True:
                self.nondominatedParticles.append(p);
                p.nondominatedPos.append(p.pos);
                
                p.pbSet.addObs(p.getCurrentPos(), True);
                self.gbSet.addObs(p.getCurrentPos(), True);
            else:
                self.dominatedParticles.append(p);

        #self.gbSet.learnProb();
        # generate new fitness
        for i in range(self.particleNum):
            p = self.particles[i];
            
            minObj = [];        

            for q in self.particles:
                if q.index == p.index:
                    continue;
                fit_delta = [];
                
                for k in range(self.objnum):
                    delta_data = self.calcObjFunc(p.pos, k) - self.calcObjFunc(q.pos, k);
                    fit_delta.append(delta_data);
                minObj.append(np.amin(fit_delta));
            
            p.fitness = np.amax(minObj);
             
            if self.firstRun == True:
                p.localbestFitness = p.fitness;
            
            
            if p.fitness < p.localbestFitness:
                p.localbestFitness = p.fitness
                p.localbestFitnessPos = p.getCurrentPos()
                #p.localbestFitness = p.fitness;
                #p.localbestPos = p.pos;
                #localbestIdx = int(np.random.random() * len(self.nondominatedParticles));
                #p.localbestPos = self.nondominatedParticles[localbestIdx].pos;
                #localbestIdx = int(np.random.random() * len(p.nondominatedPos));
                #p.localbestPos = p.nondominatedPos[localbestIdx];
                #localbestIdx = int(np.random.random() * len(self.nondominatedSet));
                #p.localbestPos = self.nondominatedSet[localbestIdx].pos;
            '''   
            if p.nondominated == True:
                p.pbSet.learnProb();
                p.localbestPos = p.pbSet.sample();
                if p.localbestPos == None:
                    p.localbestPos = p.pos; #self.getRandomPos();
            else:
            '''
            p.localbestPos = p.localbestFitnessPos;
        
            if p.fitness < self.globalbestFitness:
                self.globalbestFitness = p.fitness;
                self.globalbestFitnessPos = p.getCurrentPos();
            
            if p.nondominated == True:
                p.personalbestPos = p.pbSet.sample();
                p.globalbestPos = self.gbSet.sample();
                if p.globalbestPos == None:
                    p.globlbestPos = self.getRandomPos();
            
        #self.gbSet.clearObs();
        self.globalbestPos = self.globalbestFitnessPos;
        
        self.logger.log();    
                                        
       
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
            pos[0,d] = (np.random.random() - 0.5) * self.worldrange[d];
        return pos;
                
            