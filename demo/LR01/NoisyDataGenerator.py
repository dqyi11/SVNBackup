import numpy as np
import matplotlib.pyplot as plt

class LinearNoisyDataGenerator(object):

    def __init__(self, K, b):
        
        self.K = K
        self.b = b
        
        self.noisyDataXs = []
        self.noisyDataYs = []
        
    def generateData(self, dataRange, dataSize, var):
        
        self.noisyDataXs = np.random.random(dataSize) * (dataRange[1]-dataRange[0]) + dataRange[0]
        vars = np.random.random(dataSize) * var
        self.noisyDataYs = self.K * self.noisyDataXs + self.b + vars
        
    def plot(self):
        
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.plot(self.noisyDataXs, self.noisyDataYs, '.')
        plt.show()
        
    def dump(self, filename):
        
        lnNum = len(self.noisyDataXs)
        with open(filename, 'w') as f:
            for i in range(lnNum):
                f.write(str(self.noisyDataXs[i])+","+str(self.noisyDataYs[i])+"\n")
        
        
            
            
        

        