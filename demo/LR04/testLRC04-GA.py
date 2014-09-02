from LinearRegressionCalculator import *
import matplotlib.pyplot as plt

if __name__ == '__main__':
    
    testName = "LR04-testLRC04-GA"
    
    lr = LinearRegressionCalculator(6)
    lr.load('yacht-norm.csv')
    lr.runCnt = 5000
    lr.calcByGA(1000, [-5.0, 5.0])

    print lr.betas
    print lr.mse
    
    lr.log(testName)
    
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(np.arange(lr.runCnt), lr.fitnessVal)
    #plt.show()
    plt.savefig(testName)
    
    print lr.fitnessVal[lr.runCnt-1]
