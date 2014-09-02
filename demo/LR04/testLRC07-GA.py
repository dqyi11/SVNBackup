from NeuralNetworkCalculator import *
import matplotlib.pyplot as plt

if __name__ == '__main__':
    
    testName = "LR04-testLRC07-GA"
    
    lr = NeuralNetworkCalculator(6, 20)
    lr.load('yacht-norm.csv')
    lr.runCnt = 1000
    lr.calcByGA(2000, [-10.0, 10.0], 0.05)
    print lr.betas
    print lr.mse
    
    lr.log(testName)
    
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(np.arange(lr.runCnt), lr.fitnessVal)
    #plt.show()
    plt.savefig(testName)
    
    print lr.fitnessVal[lr.runCnt-1]