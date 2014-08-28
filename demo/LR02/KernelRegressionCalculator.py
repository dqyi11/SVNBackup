import csv
import numpy as np
from GeneticAlgorithm import *

class KernelRegressionCalculator(object):

    def __init__(self, dimension):
        self.dim = dimension
        self.dataSize = 0
        self.inputs = []
        for d in range(self.dim):
            self.inputs.append([])
        self.outputs = []
        
        self.betas = np.zeros((1, self.dim+1))
        
        self.mle = 0.0
        self.fitnessVal = []
        self.gaRunCnt = 2000

        
    
    def load(self, filename):        
        with open(filename, 'rb') as csvfile:
            reader = csv.reader(csvfile)
            for row in reader:
                self.dataSize += 1
                for d in range(self.dim):
                    self.inputs[d].append(float(row[d]))
                self.outputs.append(float(row[self.dim]))
                
        self.X = np.hstack((np.ones((self.dataSize,1)), np.array(self.inputs).T))
        self.Y = np.array(self.outputs).T