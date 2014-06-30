'''
Created on 2013-11-23

@author: Walter
'''

import numpy as np;
from Particle import *;
import matplotlib.pyplot as plt;
#import pygame as pg;
import pickle;

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
        self.paretoPeak = None;
        
        self.histPercentOfNondominance = [];
        self.histPosVariance = [];
        self.histFitVariance = [];
        self.histHausdorffDist = [];
        
        self.histSwarmCentroid = [];
        
        self.showCentroidHist = False;
        self.showAverageFitness = False;
        self.showMaximinOfCentroid = False;
        self.showGlobalBestPosition = False;
        self.showPercentOfNondominance = False;
        self.showPosVariance = False;
        self.showFitVariance = False;
        self.showHausdorffDist = False;
        
        self.showSwarmCentroid = False;
        
        
    
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
            
    def categorizeRefSet(self, loadFromFile=False, nondomSetFile=None, domSetFile=None, domAllOtherFile=None):
    
        if loadFromFile == False:
            # find dominated or nondominated
            for a in self.referenceSet:
                
                maxObj = [];
                minObj = [];
                for b in self.referenceSet:
                    if b.index == a.index:
                        continue;
                    
                    fit_delta = [];                    
                    for k in range(self.objnum):
                        a_k_obj = self.calcObjFunc(a.pos, k);
                        b_k_obj = self.calcObjFunc(b.pos, k);
                        fit_delta.append(a_k_obj - b_k_obj);
                        
                    maxObj.append(np.amax(fit_delta));
                    minObj.append(np.amin(fit_delta));

                if np.amax(minObj) <= 0:
                    self.nondominatedSet.append(a);
                else:
                    self.dominatedSet.append(a);
                    
                if np.amax(maxObj) <=0:
                    self.paretoPeak = a.pos
                    
            if self.paretoPeak == None:
                self.paretoPeak = self.getCentroidOfNondominatedSet();
                    
        else:
            self.loadNondominatedSet(nondomSetFile);
            self.loadDominatedSet(domSetFile);
            self.loadParetoPeak(domAllOtherFile);       
        
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
    def dumpParetoPeak(self, outFile):
        pickle.dump(self.paretoPeak, open(outFile, 'wb'));
    def loadNondominatedSet(self, inFile):
        self.nondominatedSet = pickle.load(open(inFile, 'rb'));
    def loadDominatedSet(self, inFile):
        self.dominatedSet = pickle.load(open(inFile, 'rb'));
    def loadParetoPeak(self, inFile):
        self.paretoPeak = pickle.load(open(inFile, 'rb'));
        
                
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
            
        for p in self.particles:
            
            '''
            U1 = np.matrix(np.zeros((self.particleDimension, self.particleDimension), np.float));
            U2 = np.matrix(np.zeros((self.particleDimension, self.particleDimension), np.float));        
            for d in range(self.particleDimension):
                U1[d,d] = np.random.random();
                U2[d,d] = np.random.random();
                
            localForce = self.phi1 * U1 * (p.localbestPos.T - p.pos.T);
            globalForce = self.phi2 * U2 * (p.globalbestPos.T - p.pos.T);
            p.vel = self.inertia * (p.vel + localForce.T + globalForce.T);
            p.pos = p.pos + p.vel;
            
            for d in range(self.particleDimension):
                if p.pos[0,d] > self.worldrange[d] / 2:
                    p.pos[0,d] = self.worldrange[d] / 2;
                elif p.pos[0,d] < - self.worldrange[d] / 2:
                    p.pos[0,d] = - self.worldrange[d] / 2;
            '''
            for d in range(self.particleDimension):
                u1 = np.random.random();
                u2 = np.random.random();
                localForce = self.phi1 * u1 * (p.localbestPos[0,d] - p.pos[0,d]);
                globalForce = self.phi2 * u2 * (p.globalbestPos[0,d] - p.pos[0,d]);
                p.vel[0,d] = self.inertia * (p.vel[0,d] + localForce + globalForce)
                p.pos[0,d] = p.pos[0,d] + p.vel[0,d];
                
                if p.pos[0,d] > self.worldrange[d] / 2:
                    p.pos[0,d] = self.worldrange[d] / 2;
                elif p.pos[0,d] < - self.worldrange[d] / 2:
                    p.pos[0,d] = - self.worldrange[d] / 2;
                    
        self.dominatedParticles = [];
        self.nondominatedParticles = [];  
        
        if self.showPosVariance == True:
            self.histPosVariance.append(self.getPosVariance());
            
        if self.showFitVariance == True:
            self.histFitVariance.append(self.getFitVariance());
            
        if self.showHausdorffDist == True:
            self.histHausdorffDist.append(self.getHausdorffDistance());
        
        self.average_fitness = np.zeros((1, self.objnum), np.float);
        for i in range(self.particleNum):
            p = self.particles[i];

            for k in range(self.objnum):
                self.average_fitness[0, k] += self.calcObjFunc(p.pos, k);
            
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
            else:
                self.dominatedParticles.append(p);
        
        for p in self.particles:        
            # find personal best
            pbIdx = int(np.random.random() * len(self.nondominatedSet));
            p.localbestPos = self.nondominatedSet[pbIdx].pos;
            
            # find global best
            #p.globalbestPos = self.paretoPeak;
            gbIdx = int(np.random.random() * len(self.nondominatedSet));
            p.globalbestPos = self.nondominatedSet[gbIdx].pos;

                          
        self.swarm_centroid = np.zeros((1,self.particleDimension), np.float);
        for p in self.particles:
            for d in range(self.particleDimension):
                self.swarm_centroid[0,d] += p.pos[0,d];
                
        self.swarm_centroid = self.swarm_centroid / self.particleNum;
        self.histSwarmCentroid.append(self.swarm_centroid);
             


        
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
    
    def getHausdorffDistance(self):
        
        distToParetoSet = []; 
        for p in self.particles:            
            distToPosInParetoSet = [];            
            for nd in self.nondominatedSet:                
                deltaDist = p.pos - nd.pos;
                distToPosInParetoSet.append(np.linalg.norm(deltaDist, ord='fro'));
            distToParetoSet.append(np.amin(distToPosInParetoSet));
        
        return np.amax(distToParetoSet);    
                
            
                
            