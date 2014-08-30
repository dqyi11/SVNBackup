from LinearRegressionCalculator import *
import matplotlib.pyplot as plt

if __name__ == '__main__':
    
    lr = LinearRegressionCalculator(4)
    lr.load('data1.csv')
    #print lr.inputs[0]
    #print lr.inputs[1]
    #print lr.inputs[2]
    #print lr.inputs[3]
    #print lr.outputs
    #lr.calc()
    lr.runCnt = 5000
    lr.calcByGA(1000, [-10.0, 10.0])
    #lr.calcByPSO(1000, [-10.0, 10.0])
    
    print "BETAS: " + str(lr.betas)
    print "MSE = " + str(lr.mse)
    
    '''
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(np.arange(lr.runCnt), lr.fitnessVal)
    plt.show()
    '''
    