import csv
import numpy as np
from GeneticAlgorithm import *

class KernelRegressionCalculator(object):

    def __init__(self, dimension, smooth_factor, kernel_func):
        self.dim = dimension
        self.dataSize = 0
        self.inputs = []
        for d in range(self.dim):
            self.inputs.append([])
        self.outputs = []
        
        self.smooth_factor = smooth_factor
        self.kernel_func = kernel_func
        
    def load(self, filename):        
        with open(filename, 'rb') as csvfile:
            reader = csv.reader(csvfile)
            for row in reader:
                self.dataSize += 1
                for d in range(self.dim):
                    self.inputs[d].append(float(row[d]))
                self.outputs.append(float(row[self.dim]))
                
        self.X = np.array(self.inputs).T
        self.Y = np.array(self.outputs)
        
        
    def calc(self):
        self.K = np.zeros((self.dataSize, self.dataSize))
        for i in range(self.dataSize):
            for j in range(self.dataSize):
                self.K[i,j] = self.kernel_func(self.X[i,:], self.X[j,:])
        self.C = np.dot( np.linalg.inv(self.K + self.smooth_factor * np.eye(self.dataSize) ) , self.Y )
        
        nY = np.zeros(self.dataSize)
        for i in range(self.dataSize):
            for j in range(self.dataSize):
                nY[i] = nY[i] + self.C[j] * self.kernel_func(self.X[j,:], self.X[i,:])
        delta = np.array(nY) - self.Y
        self.mle = np.dot(delta.T, delta) / self.dataSize 
