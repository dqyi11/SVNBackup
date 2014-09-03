import time
import numpy as np
from RegressionCalculator import *

class LinearRegressionCalculator(RegressionCalculator):

    def __init__(self, dimension):
        super(LinearRegressionCalculator, self).__init__(dimension)
        self.type = "LINEAR"
        self.betas = np.zeros(self.dim+1)
                
    def calc(self):
        
        betas = np.dot(np.dot(np.linalg.inv(np.dot(self.trainX.T, self.trainX)) , self.trainX.T) , self.trainY)
        self.betas = betas
        
        delta = self.trainY - np.dot(self.trainX, betas)
        self.trainMSE =  np.dot(delta.T, delta) / self.trainDataSize
        
    def calcLin(self, x, w):
        nX = self.trainX = np.hstack([1], np.array(x).T)
        nW = np.array(w)
        print nX.shape
        print nW.shape
        return np.dot(nX, nW)
        
    def calcByBatchGradientDescent(self, learningRate, iterationNum, batchSize, initBetas):
        
        betas = initBetas#np.random.random(self.dim+1) * (initRange[1]-initRange[0]) + initRange[0]
        for i in range(iterationNum):
            
            for j in range(0, self.trainDataSize, batchSize):
                batchDataSize = batchSize
                if self.trainDataSize - 1 - j < batchSize:
                    batchDataSize = self.trainDataSize - 1 - j
                
                deltaBetas = np.zeros(self.dim+1)
                batchDataX = np.zeros((self.dim, batchDataSize))
                batchDataY = np.zeros(batchDataSize)
                for d in range(self.dim):
                    for di in range(j, j+batchDataSize):
                        batchDataX[d, di-j] = self.trainInputs[d][di]
                        batchDataY[di-j] = self.trainOutputs[di]
                batchX = np.hstack((np.ones((batchDataSize,1)), np.array(batchDataX).T))
                batchY = np.array(batchDataY).T        
                                
                batchDelta = batchY - np.dot(batchX, betas)             
                deltaBetas[0] += learningRate * np.sum(batchDelta)
                for d in range(self.dim):
                    deltaBetas[d+1] += learningRate * np.dot(batchDelta, batchDataX[d,:])
                    
                betas += deltaBetas
                
        self.betas = betas
        
        delta = self.trainY - np.dot(self.trainX, betas)
        self.trainMSE =  np.dot(delta.T, delta) / self.trainDataSize
        
    
    def calcByStochasticGradientDescent(self, learningRate, iterationNum):
        pass

        
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
            file.write("TRAIN SIZE: " + str(self.trainDataSize) + " MSE: " + str(self.trainMSE) + "\n") 
            file.write("TEST SIZE: " + str(self.testDataSize) + " MSE: " + str(self.testMSE))
        
        
            
        
        
                    

        
                
    
            
            