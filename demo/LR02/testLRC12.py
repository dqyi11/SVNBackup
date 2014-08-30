from RBFNetworkCalculator import *
import matplotlib.pyplot as plt

if __name__ == '__main__':
    
    def kernelFunc(x1, x2):
        p1 = 1
        deltaX = np.array(x1) - np.array(x2) 
        valT = np.dot(deltaX.T, deltaX)
        val =  np.sqrt(1+valT)
        return val
    
    
    lr = RBFNetworkCalculator(7, kernelFunc)
    lr.load('auto_mpg.csv')
    #lr.calc()
    lr.runCnt = 100
    #lr.calcByGA(500, [-50.0, 50.0])
    lr.calcByPSO(500, [-50.0, 50.0])
    print lr.betas
    print lr.mle
    
    
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(np.arange(lr.runCnt), lr.fitnessVal)
    plt.show()
    
    print lr.fitnessVal[lr.runCnt-1]