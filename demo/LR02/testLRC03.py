from LinearRegressionCalculator import *
import matplotlib.pyplot as plt

if __name__ == '__main__':
    
    lr = LinearRegressionCalculator(7)
    lr.load('auto_mpg.csv')
    #print lr.inputs[0]
    #print lr.inputs[1]
    #print lr.inputs[2]
    #print lr.inputs[3]
    #print lr.outputs
    lr.calc()
    lr.calcByGA(200, [-5.0, 5.0])
    #print lr.betas
    print lr.mle
    
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(np.arange(lr.gaRunCnt), lr.fitnessVal)
    plt.show()
    
    print lr.fitnessVal[lr.gaRunCnt-1]