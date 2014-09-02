from NeuralNetworkCalculator import *
import matplotlib.pyplot as plt

if __name__ == '__main__':
    
    testName = "LR02-testLRC07-GA"
    
    lr = NeuralNetworkCalculator(6, 10)
    lr.load('yacht-norm.csv')
    lr.runCnt = 100
    lr.calcByGA(1000, [-10.0, 10.0])
    print lr.betas
    print lr.mse
    
    lr.log(testName)
    
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(np.arange(lr.runCnt), lr.fitnessVal)
    #plt.show()
    plt.savefig(testName)
    
    print lr.fitnessVal[lr.runCnt-1]