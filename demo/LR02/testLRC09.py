from KernelRegressionCalculator import *
import matplotlib.pyplot as plt

if __name__ == '__main__':
    
    def kernelFunc(x1, x2):
        p1 = 1
        deltaX = np.array(x1) - np.array(x2)
        return np.exp( - np.dot(deltaX, deltaX.T) / (2*p1**2) )
    
    lr = KernelRegressionCalculator(7, 0.5, kernelFunc)
    lr.load('auto_mpg.csv')
    lr.calc()
    #lr.runCnt = 1000
    #lr.calcByGA(1000, [-1.0, 1.0])
    #lr.calcByPSO(1000, [-1.0, 1.0])
    #print lr.betas
    print lr.mle
    
    
    '''
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(np.arange(lr.runCnt), lr.fitnessVal)
    plt.show()
    
    print lr.fitnessVal[lr.runCnt-1]
    '''