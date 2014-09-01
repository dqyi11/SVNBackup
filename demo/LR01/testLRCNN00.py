from LinearRegressionCalculator import *
import matplotlib.pyplot as plt

if __name__ == '__main__':
    
    #lr = LinearRegressionCalculator(7)
    #lr.load('auto_mpg.csv')
    lr = LinearRegressionCalculator(3)
    lr.load('nn_data.csv')
    lr.calc()
    #lr.runCnt = 1000
    #lr.calcByGA(1000, [-2.0, 2.0])
    #lr.calcByPSO(1000, [-2.0, 2.0])
    print lr.betas
    print lr.mse