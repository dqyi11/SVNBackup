from LinearRegressionCalculator import *
import matplotlib.pyplot as plt

if __name__ == '__main__':
    
    testName = "LR05-testLRC04-GBD"
    
    lr = LinearRegressionCalculator(4)
    lr.loadTrainData('data2-train.csv')
    lr.loadTestData('data2-test.csv')

    #lr.calc()
    lr.calcByBatchGradientDescent(0.01, 1000000, 1, [-5.0, 5.0])
    lr.calcTestMSE(lr.betas)
    
    print lr.betas
    print "TRAIN SIZE: " + str(lr.trainDataSize) + " MSE: " + str(lr.trainMSE)
    print "TEST SIZE: " + str(lr.testDataSize) + " MSE: " + str(lr.testMSE)

    lr.log(testName)
    

    