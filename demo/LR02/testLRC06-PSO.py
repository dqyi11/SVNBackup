from NeuralNetworkCalculator import *
import matplotlib.pyplot as plt

if __name__ == '__main__':
    
    testName = "LR02-testLRC06-PSO"
    
    lr = NeuralNetworkCalculator(7)
    lr.load('auto_mpg-norm.csv')

    lr.runCnt = 100
    lr.calcByPSO(500, [-10.0, 10.0])
    print lr.betas
    print lr.mle
    
    lr.log(testName)
    
    
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(np.arange(lr.runCnt), lr.fitnessVal)
    #plt.show()
    plt.savefig(testName)
    
    print lr.fitnessVal[lr.runCnt-1]