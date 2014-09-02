import time
import numpy as np
from RegressionCalculator import *

class LinearRegressionCalculator(RegressionCalculator):

    def __init__(self, dimension):
        super(LinearRegressionCalculator, self).__init__(dimension)
                
    def calc(self):
        
        betas = np.dot(np.dot(np.linalg.inv(np.dot(self.trainX.T, self.trainX)) , self.trainX.T) , self.trainY)
        self.betas = betas
        
        delta = self.trainY - np.dot(self.trainX, betas)
        self.trainMSE =  np.dot(delta.T, delta) / self.trainDataSize
        
    def calcFitness(self, weight):
        beta = np.array(weight)
        delta = self.trainY - np.dot(self.trainX, beta)
        return np.dot(delta.T, delta) / self.trainDataSize 
    
    
    def calcTestMSE(self, weight):
        beta = np.array(weight)
        delta = self.testY - np.dot(self.testX, beta)
        self.testMSE = np.dot(delta.T, delta) / self.testDataSize
      
        
    def log(self, filename):
        
        id = str(time.time())
        with open(filename+"-"+id+".txt", 'w') as file:
            paramStr = "PARAM: "
            for b in self.betas:
                paramStr +=  str(b) + " "
            paramStr += "\n"
            file.write(paramStr)
            
            for fVal in self.trainFitnessVal:
                file.write(str(fVal) + "\n")
                
            file.write("\n")
            file.write("TRAIN SIZE: " + str(self.trainDataSize) + " MSE: " + str(self.trainMSE))
            file.write("TEST SIZE: " + str(self.testDataSize) + " MSE: " + str(self.testMSE))
        
        
            
        
        
                    

        
                
    
            
            