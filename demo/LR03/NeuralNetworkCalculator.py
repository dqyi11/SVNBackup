import csv
import time
import numpy as np
from GeneticAlgorithm import *
from NeuralNetwork import *

class NeuralNetworkCalculator(object):

    def __init__(self, dimension, hiddenNum):
        self.dim = dimension
        self.dataSize = 0
        self.inputs = []
        for d in range(self.dim):
            self.inputs.append([])
        self.outputs = []
               
        self.mse = 0.0
        self.fitnessVal = []
        self.runCnt = 2000
        self.nn = NeuralNetwork([self.dim, hiddenNum, 1])
        
        self.ga = None

    
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

        nY = []
        for i in range(self.dataSize):
            x = self.X[i,:]
            nY.append(self.nn.calcFunc(weight, x)[0])
        delta = self.Y - np.array(nY)
        return np.dot(delta.T, delta) / self.dataSize 
        
    def calcByGA(self, population_num, geneRange, mutateVar):
        chromoLen = self.nn.weight_num + self.nn.bias_num
        
        self.ga = GeneticAlgorithm(population_num, geneRange, chromoLen, self.calcFitness, mutateVar)
        
        self.fitnessVal = []
        for t in range(self.runCnt):
            self.ga.next()
            self.fitnessVal.append(self.ga.population[0].fitness)
            print str(t) + " : " + str(self.ga.population[0].fitness)
            
        self.betas = np.array(self.ga.population[0].genes)
        nY = []
        for i in range(self.dataSize):
            x = self.X[i,:]
            nY.append(self.nn.calcFunc(self.betas, x)[0])
        delta = self.Y - np.array(nY)
        self.mse = np.dot(delta.T, delta) / self.dataSize 
        
    def log(self, filename):
        
        id = str(time.time())
        with open(filename+"-"+id+".txt", 'w') as file:
            paramStr = "PARAM: "
            paramStr += "I: " + str(self.nn.input_num) + " "
            paramStr += "H: " + str(self.nn.hidden_num) + " "
            paramStr += "O: " + str(self.nn.output_num) + " "
            paramStr += "\n"
            file.write(paramStr)
            
            if self.ga != None:
                gaStr = "GA "
                gaStr += "Num: " + str(self.ga.population_num) + " "
                gaStr += "Range: " + str(self.ga.geneRange) + " "
                gaStr += "mutateVar: " + str(self.ga.mutateVar) + " "
                gaStr += "mutateProb: " + str(self.ga.mutateProb) + "\n"
                file.write(gaStr)
            
            for fVal in self.fitnessVal:
                file.write(str(fVal) + "\n")
            
