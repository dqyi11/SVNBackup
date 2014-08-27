from NoisyDataGenerator import *

if __name__ == '__main__':
    
    gnr = LinearNoisyDataGenerator(2.0, 1.0)
    gnr.generateData([0.1, 5.0], 200, 2.0)
    gnr.plot()
    gnr.dump('testData-200.csv')