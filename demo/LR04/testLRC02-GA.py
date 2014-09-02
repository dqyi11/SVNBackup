from LinearRegressionCalculator import *
import matplotlib.pyplot as plt

if __name__ == '__main__':
    
    testName = "LR04-testLRC02-GA"
    
    lr = LinearRegressionCalculator(4)
    lr.load('data1.csv')

    lr.runCnt = 5000
    lr.calcByGA(1000, [-10.0, 10.0])
    
    print "BETAS: " + str(lr.betas)
    print "MSE = " + str(lr.mse)
    
    lr.log(testName)
    
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(np.arange(lr.runCnt), lr.fitnessVal)
    #plt.show()
    plt.savefig(testName)
    
    print lr.fitnessVal[lr.runCnt-1]
    