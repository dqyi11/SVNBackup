from LinearRegressionCalculator import *
import matplotlib.pyplot as plt

if __name__ == '__main__':
    
    lr = LinearRegressionCalculator(7)
    lr.load('auto_mpg-norm.csv')
    lr.calc()

    print lr.betas
    print lr.mse
    