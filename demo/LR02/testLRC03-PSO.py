from LinearRegressionCalculator import *
import matplotlib.pyplot as plt

if __name__ == '__main__':
    
    testName = "LR02-testLRC03-PSO"
    
    lr = LinearRegressionCalculator(7)
    lr.load('auto_mpg-norm.csv')

    lr.runCnt = 1000
    lr.calcByPSO(2000, [-1.0, 1.0])
    print lr.betas
    print lr.mse
    
    lr.log(testName)
    
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(np.arange(lr.runCnt), lr.fitnessVal)
    #plt.show()
    
    plt.savefig(testName)
    
    print lr.fitnessVal[lr.runCnt-1]