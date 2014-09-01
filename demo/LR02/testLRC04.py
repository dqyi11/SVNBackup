from LinearRegressionCalculator import *
import matplotlib.pyplot as plt

if __name__ == '__main__':
    
    lr = LinearRegressionCalculator(6)
    lr.load('yacht-norm.csv')
    lr.calc()

    print lr.betas
    print lr.mse
    
