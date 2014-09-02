from LinearRegressionCalculator import *
import matplotlib.pyplot as plt

if __name__ == '__main__':
    
    testName = "LR05-testLRC01-GBD"
    
    lr = LinearRegressionCalculator(1)
    lr.loadTrainData('testData-30-train.csv')
    lr.loadTestData('testData-30-test.csv')
    #lr.calc()
    lr.calcByBatchGradientDescent(0.01, 100, 3, [-1.0, 1.0])
    lr.calcTestMSE(lr.betas)
    
    print lr.betas
    print "TRAIN SIZE: " + str(lr.trainDataSize) + " MSE: " + str(lr.trainMSE)
    print "TEST SIZE: " + str(lr.testDataSize) + " MSE: " + str(lr.testMSE)

    lr.log(testName)
    
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(lr.trainInputs[0], lr.trainOutputs, '.')
    maxVal = np.max(lr.trainInputs[0])
    minVal = np.min(lr.trainInputs[0])
    xs = np.arange(minVal, maxVal+0.01, 0.01)
    ys = xs * lr.betas[1] + lr.betas[0]
    ax.plot(xs, ys)
    title_str = "Train M.S.E = " + str(lr.trainMSE) + "\n"
    title_str = "Test M.S.E = " + str(lr.testMSE) + "\n"
    title_str += "  beta 0 = " + str(lr.betas[0])
    title_str += ", beta 1 = " + str(lr.betas[1])
    ax.set_title(title_str)
    #plt.show()
    plt.savefig(testName)
    