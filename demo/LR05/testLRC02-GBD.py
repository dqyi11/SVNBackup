from LinearRegressionCalculator import *
import matplotlib.pyplot as plt

if __name__ == '__main__':
    
    testName = "LR05-testLRC02-GBD"
    
    lr = LinearRegressionCalculator(4)
    lr.loadTrainData('data1-train.csv')
    lr.loadTestData('data1-test.csv')

    lr.calcByBatchGradientDescent(0.0001, 1000000, 2, [80.0, 5.0, 5.0, 5.0, 5.0])
    lr.calcTestMSE(lr.betas)
    
    print lr.betas
    print "TRAIN SIZE: " + str(lr.trainDataSize) + " MSE: " + str(lr.trainMSE)
    print "TEST SIZE: " + str(lr.testDataSize) + " MSE: " + str(lr.testMSE)

    lr.log(testName)