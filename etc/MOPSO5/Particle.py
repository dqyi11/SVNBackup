'''
Created on 2013-11-23

@author: Walter
'''
import numpy as np;
from PersonalBestSet import *;

class Particle(object):

    def __init__(self, index, dimension):
        self.index = index;
        self.dimension = dimension;
        self.pos = np.matrix(np.zeros((1,dimension), np.float));
        self.vel = np.matrix(np.zeros((1,dimension), np.float));
        self.localbestFitness = 0.0;
        self.localbestFitnessPos = self.pos;
        self.localbestPos = self.pos;
        self.globalbestPos = self.pos;
        self.forceDirection = np.matrix([0.0, 0.0], np.float);
        self.nondominated = False;
        self.fitness = 0.0;
        self.nondominatedPos = [];
        self.pbSet = PersonalBestSet(index, dimension);
        
class Reference(object):
    
    def __init__(self, dimension, index):
        self.nondominated = True;
        self.pos = np.matrix((1, dimension), np.float);
        self.index = index;