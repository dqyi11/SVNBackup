'''
Created on 2013-11-23

@author: Walter
'''
import numpy as np;

class Particle(object):

    def __init__(self, index, dimension):
        self.index = index;
        self.dimension = dimension;
        self.pos = np.matrix(np.zeros((1,dimension), np.float));
        self.vel = np.matrix(np.zeros((1,dimension), np.float));
        self.localbestFitness = 0.0;
        self.localbestPos = self.pos;
        self.forceDirection = np.matrix([0.0, 0.0], np.float);
        self.nondominated = False;
        
class Reference(object):
    
    def __init__(self):
        self.nondominated = True;