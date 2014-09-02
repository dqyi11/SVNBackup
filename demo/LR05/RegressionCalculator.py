import csv
import numpy as np
from GeneticAlgorithm import *
from ParticleSwarmOptimization import *


class RegressionCalculator(object):


    def __init__(self, dimension):
        
        self.dim = dimension
        self.trainDataSize = 0
        self.trainInputs = []
        self.testInputs = []
        for d in range(self.dim):
            self.trainInputs.append([])
            self.testInputs.append([])
        self.trainOutputs = []
        self.testOutputs = []
        
        self.trainMSE = 0.0
        self.testMSE = 0.0
        self.trainFitnessVal = []
        self.runCnt = 2000
        
        self.ga = None
        self.pso = None
        
        self.type = None

    
    def loadTrainData(self, filename):
        self.trainDataSize = 0        
        with open(filename, 'rb') as csvfile:
            reader = csv.reader(csvfile)
            for row in reader:
                self.trainDataSize += 1
                for d in range(self.dim):
                    self.trainInputs[d].append(float(row[d]))
                self.trainOutputs.append(float(row[self.dim]))
        if self.type == "LINEAR":      
            self.trainX = np.hstack((np.ones((self.trainDataSize,1)), np.array(self.trainInputs).T))
            self.trainY = np.array(self.trainOutputs).T
        elif self.type == "NEURAL_NET":
            self.trainX = np.array(self.trainInputs).T
            self.trainY = np.array(self.trainOutputs)
       
    def loadTestData(self, filename):
        self.testDataSize = 0        
        with open(filename, 'rb') as csvfile:
            reader = csv.reader(csvfile)
            for row in reader:
                self.testDataSize += 1
                for d in range(self.dim):
                    self.testInputs[d].append(float(row[d]))
                self.testOutputs.append(float(row[self.dim]))
        if self.type == "LINEAR":        
            self.testX = np.hstack((np.ones((self.testDataSize,1)), np.array(self.testInputs).T))
            self.testY = np.array(self.testOutputs).T
        elif self.type == "NEURAL_NET":
            self.trainX = np.array(self.trainInputs).T
            self.trainY = np.array(self.trainOutputs)        
        
        
    def calcByGA(self, population_num, geneRange, mutateVar):
        
        if self.type == "LINEAR":
            chromoLen = self.dim + 1
        elif self.type == "NEURAL_NET":
            chromoLen = self.nn.weight_num + self.nn.bias_num
        
        self.ga = GeneticAlgorithm(population_num, geneRange, chromoLen, self.calcFitness, mutateVar)
        
        self.trainFitnessVal = []
        for t in range(self.runCnt):
            self.ga.next()
            self.trainFitnessVal.append(self.ga.population[0].fitness)
            print str(t) + " : " + str(self.ga.population[0].fitness)
            
        self.betas = np.array(self.ga.population[0].genes)
        if self.type == "LINEAR":
            delta = self.trainY - np.dot(self.trainX, self.betas)
        elif self.type == "NEURAL_NET":
            nY = []
            for i in range(self.trainDataSize):
                x = self.trainX[i,:]
                nY.append(self.nn.calcFunc(self.betas, x)[0])
            delta = self.trainY - np.array(nY)    
            
        self.trainMSE = np.dot(delta.T, delta) / self.trainDataSize
        
    def calcByPSO(self, population_num, geneRange):

        if self.type == "LINEAR":
            particleDim = self.dim + 1
        elif self.type == "NEURAL_NET":
            particleDim = self.nn.weight_num + self.nn.bias_num
        
        self.pso = Swarm(population_num, particleDim, geneRange, self.calcFitness, 0.4, 1.0, 1.0)
        
        self.trainFitnessVal = []
        for t in range(self.runCnt):
            self.pso.next()
            self.trainFitnessVal.append(self.pso.gbFitness)
            print str(t) + " : " + str(self.pso.gbFitness)
            
        self.betas = np.array(self.pso.gb)
        if self.type == "LINEAR":
            delta = self.trainY - np.dot(self.trainX, self.betas)
        elif self.type == "NEURAL_NET":
            nY = []
            for i in range(self.trainDataSize):
                x = self.trainX[i,:]
                nY.append(self.nn.calcFunc(self.betas, x)[0])
            delta = self.trainY - np.array(nY)
            
        self.trainMSE = np.dot(delta.T, delta) / self.trainDataSize  