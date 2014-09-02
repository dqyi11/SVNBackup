from LinearRegressionCalculator import *
import matplotlib.pyplot as plt

if __name__ == '__main__':
    
    testName = "LR04-testLRC03-GA"
    
    lr = LinearRegressionCalculator(7)
    lr.load('auto_mpg-norm.csv')

    lr.runCnt = 1000
    lr.calcByGA(1000, [-1.0, 1.0], 0.001)

    print lr.betas
    print lr.mse
    
    lr.log(testName)
    
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(np.arange(lr.runCnt), lr.fitnessVal)
    #plt.show()
    plt.savefig(testName)
    
    print lr.fitnessVal[lr.runCnt-1]
    