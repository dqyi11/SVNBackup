import csv
import numpy as np
from GeneticAlgorithm import *

class LinearRegressionCalculator(object):

    def __init__(self, dimension):
        self.dim = dimension
        self.dataSize = 0
        self.inputs = []
        for d in range(self.dim):
            self.inputs.append([])
        self.outputs = []
        
        self.betas = np.zeros((1, self.dim+1))
        
        self.mse = 0.0
        self.fitnessVal = []
        self.runCnt = 2000
 
    
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
                
    def calc(self):
        
        betas = np.dot(np.dot(np.linalg.inv(np.dot(self.X.T, self.X)) , self.X.T) , self.Y)
        self.betas = betas
        
        delta = self.Y - np.dot(self.X, betas)
        self.mse =  np.dot(delta.T, delta) / self.dataSize
        
    def calcFitness(self, weight):
        beta = np.array(weight)

        delta = self.Y - np.dot(self.X, beta)
        return np.dot(delta.T, delta) / self.dataSize 
        
    def calcByGA(self, population_num, geneRange, mutateVar):
        chromoLen = self.dim + 1
        
        ga = GeneticAlgorithm(population_num, geneRange, chromoLen, self.calcFitness, mutateVar)
        
        self.fitnessVal = []
        for t in range(self.runCnt):
            ga.next()
            self.fitnessVal.append(ga.population[0].fitness)
            print str(t) + " : " + str(ga.population[0].fitness)
            
        self.betas = np.array(ga.population[0].genes)
        delta = self.Y - np.dot(self.X, self.betas)

        self.mse = np.dot(delta.T, delta) / self.dataSize

        
            
        
        
                    

        
                
    
            
            