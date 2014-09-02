from LinearRegressionCalculator import *
import matplotlib.pyplot as plt

if __name__ == '__main__':
    
    testName = "LR05-testLRC03"
    
    lr = LinearRegressionCalculator(4)
    lr.loadTrainData('auto_mpg-train.csv')
    lr.loadTestData('auto_mpg-test.csv')
    lr.calc()
    lr.calcTestMSE(lr.betas)
    
    print lr.betas
    print "TRAIN SIZE: " + str(lr.trainDataSize) + " MSE: " + str(lr.trainMSE)
    print "TEST SIZE: " + str(lr.testDataSize) + " MSE: " + str(lr.testMSE)
    
    lr.log(testName)
