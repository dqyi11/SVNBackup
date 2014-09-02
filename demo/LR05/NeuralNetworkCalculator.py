import time
import numpy as np
from NeuralNetwork import *
from RegressionCalculator import *

class NeuralNetworkCalculator(RegressionCalculator):

    def __init__(self, dimension, hiddenNum):
        super(NeuralNetworkCalculator, self).__init__(dimension)
        self.nn = NeuralNetwork([self.dim, hiddenNum, 1])
        self.type = "NEURAL_NET"
        

    def calcFitness(self, weight):

        nY = []
        for i in range(self.trainDataSize):
            x = self.trainX[i,:]
            nY.append(self.nn.calcFunc(weight, x)[0])
        delta = self.trainY - np.array(nY)
        return np.dot(delta.T, delta) / self.trainDataSize
    
    def calcTestMSE(self, weight):
        
        nTestY = []
        for i in range(self.testDataSize):
            x = self.testX[i,:]
            nTestY.append(self.nn.calcFunc(weight, x)[0])
        delta = self.testY - np.array(nTestY)
        self.testMSE = np.dot(delta.T, delta) / self.testDataSize     
        
    def log(self, filename):
        
        id = str(time.time())
        with open(filename+"-"+id+".txt", 'w') as file:
            paramStr = "PARAM: "
            paramStr += "I: " + str(self.nn.input_num) + " "
            paramStr += "H: " + str(self.nn.hidden_num) + " "
            paramStr += "O: " + str(self.nn.output_num) + " "
            paramStr += "\n"
            file.write(paramStr)
            
            if self.pso != None:
                psoStr = "PSO "
                psoStr += "Num: " + str(self.pso.particle_num) + " "
                psoStr += "Range: " + str(self.pso.posRange) + " "
                psoStr += "Chi: " + str(self.pso.chi) + " "
                psoStr += "Phi_G: " + str(self.pso.phi_g) + " "
                psoStr += "Phi_P: " + str(self.pso.phi_p) + "\n"
                file.write(psoStr)
            elif self.ga != None:
                gaStr = "GA "
                gaStr += "Num: " + str(self.ga.population_num) + " "
                gaStr += "Range: " + str(self.ga.geneRange) + "\n"
                file.write(gaStr)
            
            for fVal in self.trainFitnessVal:
                file.write(str(fVal) + "\n")
                
            file.write("\n")
            file.write("TRAIN SIZE: " + str(self.trainDataSize) + " MSE: " + str(self.trainMSE))
            file.write("TEST SIZE: " + str(self.testDataSize) + " MSE: " + str(self.testMSE))
            
