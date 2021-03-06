from LinearRegressionCalculator import *
import matplotlib.pyplot as plt

if __name__ == '__main__':
    
    testName = "LR05-testLRC03-GBD"
    
    lr = LinearRegressionCalculator(7)
    lr.loadTrainData('auto_mpg-train.csv')
    lr.loadTestData('auto_mpg-test.csv')

    #lr.calc()
    lr.calcByBatchGradientDescent(0.0000001, 1000000, 1, [-5.0, 5.0])
    lr.calcTestMSE(lr.betas)
    
    print lr.betas
    print "TRAIN SIZE: " + str(lr.trainDataSize) + " MSE: " + str(lr.trainMSE)
    print "TEST SIZE: " + str(lr.testDataSize) + " MSE: " + str(lr.testMSE)

    lr.log(testName)
    

    