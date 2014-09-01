from NeuralNetworkCalculator import *
import matplotlib.pyplot as plt

if __name__ == '__main__':
    
    lr = NeuralNetworkCalculator(3)
    lr.load('nn_data.csv')
    lr.runCnt = 100
    lr.calcByGA(500, [-10.0, 10.0])
    print lr.betas
    print lr.mle
    
    
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(np.arange(lr.runCnt), lr.fitnessVal)
    plt.show()
    
    print lr.fitnessVal[lr.runCnt-1]