from NeuralNetworkCalculator import *
import matplotlib.pyplot as plt

if __name__ == '__main__':
    
    lr = NeuralNetworkCalculator(7)
    lr.load('auto_mpg-norm.csv')
    lr.runCnt = 500
    lr.calcByGA(2000, [-10.0, 10.0])
    print lr.betas
    print lr.mse
    
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(np.arange(lr.runCnt), lr.fitnessVal)
    plt.show()
    
    print lr.fitnessVal[lr.runCnt-1]
