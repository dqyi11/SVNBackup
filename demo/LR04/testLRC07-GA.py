from NeuralNetworkCalculator import *
import matplotlib.pyplot as plt

if __name__ == '__main__':
    
    lr = NeuralNetworkCalculator(6, 10)
    lr.load('yacht-norm.csv')
    lr.runCnt = 100
    lr.calcByGA(1000, [-10.0, 10.0], 0.05)
    print lr.betas
    print lr.mse
    
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(np.arange(lr.runCnt), lr.fitnessVal)
    plt.show()
    
    print lr.fitnessVal[lr.runCnt-1]