from NeuralNetwork import *
import csv

class NeuralNetworkDataGenerator(object):

    def __init__(self, shape, range):
        self.input_num = shape[0]
        self.hidden_num = shape[1]
        self.output_num = shape[2]
        
        self.range = range
        self.nn = NeuralNetwork(shape)
        self.weights = np.random.random(self.nn.weight_num+self.nn.bias_num) * (range[1] - range[0]) + range[0]
        self.dataSize = 0  
    
    def geneData(self, dataSize, inputRange):
        self.dataSize = dataSize
        self.X = np.zeros((self.dataSize, self.input_num))
        self.Y = np.zeros((self.dataSize, self.output_num))
        for d in range(self.input_num):
            rndVal = np.random.random(self.dataSize) * (inputRange[d][1] - inputRange[d][0]) + inputRange[d][0]
            for i in range(self.dataSize):
                self.X[i, d] = rndVal[i]
        
        for i in range(self.dataSize):
            self.Y[i,0] = self.nn.calcFunc(self.weights, self.X[i,:])[0]
    
    def dump(self, filename):
        
        with open(filename+".txt", "w") as file1:
            for i in range(self.nn.weight_num+self.nn.bias_num):
                file1.write(str(self.weights[i])+"\n")
                
        with open(filename+".csv", "w") as file2:
            for i in range(self.dataSize):
                dataTxt = ""
                for d in range(self.input_num):
                    dataTxt += str(self.X[i,d]) +", "
                dataTxt += str(self.Y[i,0])
                file2.write(dataTxt+"\n")
                
    
        
        
        