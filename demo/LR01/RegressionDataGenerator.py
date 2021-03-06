import numpy as np

class RegressionDataGenerator(object):


    def __init__(self, shape, func):
        self.input_dim = shape[0]
        self.output_dim = shape[1]
        self.func = func
        self.dataSize = 0
        
    def geneData(self, dataSize, var):
        self.dataSize = dataSize
        self.X = np.zeros((self.dataSize, self.input_dim))
        self.Y = np.zeros((self.dataSize, self.output_dim))
        for i in range(self.dataSize):
            inVal = np.random.random(self.input_dim)
            outVal = self.func(inVal) + np.random.normal(0, var, self.output_dim)
            for d in range(self.input_dim):
                self.X[i, d] = inVal[d]
            for d in range(self.output_dim):
                self.Y[i, d] = outVal[d]
                
        
    def dump(self, filename):
        
        with open(filename+".csv", "wb") as file:
            for i in range(self.dataSize):
                dataTxt = ""
                for id in range(self.input_dim):
                    dataTxt += str(self.X[i,id]) + ","
                if self.output_dim == 1:
                    dataTxt += str(self.Y[i, 0])
                else:
                    for od in range(self.output_dim - 1):
                        dataTxt += str(self.Y[i, od]) + ","
                    dataTxt += str(self.Y[i, self.output_dim-1])
                dataTxt += "\n"
                
                file.write(dataTxt)
            
 
        