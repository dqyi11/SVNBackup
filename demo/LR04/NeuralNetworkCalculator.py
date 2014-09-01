import csv
import numpy as np
from GeneticAlgorithm import *
from NeuralNetwork import *

class NeuralNetworkCalculator(object):

    def __init__(self, dimension):
        self.dim = dimension
        self.dataSize = 0
        self.inputs = []
        for d in range(self.dim):
            self.inputs.append([])
        self.outputs = []
        
        #self.betas = np.zeros((1, self.dim+1))
        
        self.mle = 0.0
        self.fitnessVal = []
        self.runCnt = 2000
        self.nn = NeuralNetwork([self.dim, 10, 1])

    
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
        
        
    def calcFitness(self, weight):
        #beta = np.array(weight)
        #print beta.shape
        #print self.X.shape
        #print self.Y.shape
        nY = []
        for i in range(self.dataSize):
            x = self.X[i,:]
            nY.append(self.nn.calcFunc(weight, x)[0])
        delta = self.Y - np.array(nY)
        return np.dot(delta.T, delta) / self.dataSize 
        
    def calcByGA(self, population_num, geneRange):
        chromoLen = self.nn.weight_num + self.nn.bias_num
        
        #self.X = np.hstack((np.ones((self.dataSize,1)), np.array(self.inputs).T))
        #self.Y = np.array(self.outputs).T
        
        ga = GeneticAlgorithm(population_num, geneRange, chromoLen, self.calcFitness)
        
        self.fitnessVal = []
        for t in range(self.runCnt):
            ga.next()
            self.fitnessVal.append(ga.population[0].fitness)
            print str(t) + " : " + str(ga.population[0].fitness)
            
        self.betas = np.array(ga.population[0].genes)
        nY = []
        for i in range(self.dataSize):
            x = self.X[i,:]
            nY.append(self.nn.calcFunc(self.betas, x)[0])
        delta = self.Y - np.array(nY)
        self.mle = np.dot(delta.T, delta) / self.dataSize 
        

            
