from NeuralNetworkCalculator import *
import matplotlib.pyplot as plt

if __name__ == '__main__':
    
    #lr = NeuralNetworkCalculator(7)
    #lr.load('auto_mpg-norm.csv')
    lr = NeuralNetworkCalculator(1)
    lr.load('mlp_reg.csv')
    #lr.calc()
    lr.runCnt = 100
    lr.calcByGA(1000, [-10.0, 10.0])
    #lr.calcByPSO(1000, [-5.0, 5.0])
    print lr.betas
    print lr.mle
    
    
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(np.arange(lr.runCnt), lr.fitnessVal)
    plt.show()
    
    print lr.fitnessVal[lr.runCnt-1]