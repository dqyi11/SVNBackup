from LinearRegressionCalculator import *
import matplotlib.pyplot as plt

if __name__ == '__main__':
    
    testName = "LR02-testLRC02"
    
    lr = LinearRegressionCalculator(4)
    lr.load('data1.csv')

    lr.calc()
    
    
    print "BETAS: " + str(lr.betas)
    print "MSE = " + str(lr.mse)
    
