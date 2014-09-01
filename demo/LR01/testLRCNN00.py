from LinearRegressionCalculator import *
import matplotlib.pyplot as plt

if __name__ == '__main__':
    
    lr = LinearRegressionCalculator(3)
    lr.load('nn_data.csv')
    lr.calc()
    print lr.betas
    print lr.mse