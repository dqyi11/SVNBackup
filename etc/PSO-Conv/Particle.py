'''
Created on 2013-11-23

@author: Walter
'''
import numpy as np;

class Particle(object):

    def __init__(self, index, dimension):
        self.index = index
        self.dimension = dimension
        self.pos = np.matrix(np.zeros((1,dimension), np.float))
        self.vel = np.matrix(np.zeros((1,dimension), np.float))
        self.localbestFitness = 0.0
        self.localbestPos = self.pos
        self.fitness = 0.0

    def getCurrentPos(self):
        newPos = np.matrix(np.zeros((1,self.dimension), np.float));
        for d in range(self.dimension):
            newPos[0,d] = self.pos[0,d];
        return newPos;