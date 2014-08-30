from NeuralNetworkDataGenerator import *

if __name__ == '__main__':
    
    gnr = NeuralNetworkDataGenerator([3,5,1], [-5.0, 5.0])
    gnr.geneData(300, [[-5.0, 5.0], [-1.0, 1.0], [-10.0, 10.0]])
    gnr.dump("nn_data")