from LinearRegressionCalculator import *
import matplotlib.pyplot as plt

if __name__ == '__main__':
    
    lr = LinearRegressionCalculator(7)
    lr.load('auto_mpg-norm.csv')

    lr.runCnt = 1000
    lr.calcByGA(1000, [-2.0, 2.0])

    print lr.betas
    print lr.mse
    
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(np.arange(lr.runCnt), lr.fitnessVal)
    plt.show()
    
    print lr.fitnessVal[lr.runCnt-1]
    