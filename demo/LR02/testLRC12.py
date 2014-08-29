from RBFNetworkCalculator import *
import matplotlib.pyplot as plt

if __name__ == '__main__':
    
    def kernelFunc(x1, x2):
        p1 = 1
        deltaX = np.linalg.norm( np.array(x1) - np.array(x2) )
        
        return np.exp( - 2 * deltaX**2 )
    
    
    lr = RBFNetworkCalculator(7, kernelFunc)
    lr.load('auto_mpg.csv')
    #lr.calc()
    lr.runCnt = 100
    lr.calcByGA(500, [-10.0, 10.0])
    #lr.calcByPSO(500, [-10.0, 10.0])
    print lr.betas
    print lr.mle
    
    
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(np.arange(lr.runCnt), lr.fitnessVal)
    plt.show()
    
    print lr.fitnessVal[lr.runCnt-1]