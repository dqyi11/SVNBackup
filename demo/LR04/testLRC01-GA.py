from LinearRegressionCalculator import *
import matplotlib.pyplot as plt

if __name__ == '__main__':
    
    testName = "LR04-testLRC01-GA"
    
    lr = LinearRegressionCalculator(1)
    lr.load('testData-20.csv')
    
    lr.runCnt = 100
    lr.calcByGA(800, [-5.0, 5.0])
    
    print lr.betas
    print lr.mse
    
    lr.log(testName)
    
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(lr.inputs[0], lr.outputs, '.')
    maxVal = np.max(lr.inputs[0])
    minVal = np.min(lr.inputs[0])
    xs = np.arange(minVal, maxVal+0.01, 0.01)
    ys = xs * lr.betas[1] + lr.betas[0]
    ax.plot(xs, ys)
    title_str = "M.S.E = " + str(lr.mse) + "\n"
    title_str += "  beta 0 = " + str(lr.betas[0])
    title_str += ", beta 1 = " + str(lr.betas[1])
    ax.set_title(title_str)
    
    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111)
    ax2.plot(np.arange(lr.runCnt), lr.fitnessVal)
    
    print lr.fitnessVal[lr.runCnt-1]
    
    #plt.show()
    plt.savefig(testName)
    