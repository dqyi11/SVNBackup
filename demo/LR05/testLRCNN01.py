from LinearRegressionCalculator import *
import matplotlib.pyplot as plt

if __name__ == '__main__':
    
    testName = "LR05-testLRCNN01"
    
    lr = LinearRegressionCalculator(3)
    lr.loadTrainData('nn_data-train.csv')
    lr.loadTestData('nn_data-test.csv')
    lr.calc()
    lr.calcTestMSE(lr.betas)
    
    print lr.betas
    print "TRAIN SIZE: " + str(lr.trainDataSize) + " MSE: " + str(lr.trainMSE)
    print "TEST SIZE: " + str(lr.testDataSize) + " MSE: " + str(lr.testMSE)
    
    lr.log(testName)