from LinearRegressionCalculator import *
import matplotlib.pyplot as plt

if __name__ == '__main__':
    
    lr = LinearRegressionCalculator(1)
    lr.load('testData-200.csv')
    
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(lr.inputs[0], lr.outputs, '.')
    plt.show()