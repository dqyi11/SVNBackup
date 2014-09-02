from LinearRegressionCalculator import *
import matplotlib.pyplot as plt

if __name__ == '__main__':
    
    testName = "LR05-testLRC03-GA"
    
    lr = LinearRegressionCalculator(4)
    lr.loadTrainData('auto_mpg-train.csv')
    lr.loadTestData('auto_mpg-test.csv')

    lr.runCnt = 1000
    lr.calcByGA(4000, [-10.0, 10.0], 1.0)
    lr.calcTestMSE(lr.betas)
    
    print lr.betas
    print "TRAIN SIZE: " + str(lr.trainDataSize) + " MSE: " + str(lr.trainMSE)
    print "TEST SIZE: " + str(lr.testDataSize) + " MSE: " + str(lr.testMSE)
    
    lr.log(testName)
    
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(np.arange(lr.runCnt), lr.trainFitnessVal)
    ax.set_xlabel("Iteration")
    ax.set_ylabel("Fitness")
    title_str = "Train M.S.E = " + str(lr.trainMSE) + " "
    title_str += "Test M.S.E = " + str(lr.testMSE) + "\n"
    title_str += "  beta 0 = " + str(lr.betas[0])
    title_str += ", beta 1 = " + str(lr.betas[1])
    ax.set_title(title_str)
    print lr.trainFitnessVal[lr.runCnt-1]
    
    #plt.show()
    plt.savefig(testName)
    