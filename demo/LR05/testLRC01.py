from LinearRegressionCalculator import *
import matplotlib.pyplot as plt

if __name__ == '__main__':
    
    testName = "LR05-testLRC01"
    
    lr = LinearRegressionCalculator(1)
    lr.load('testData-20.csv')
    lr.calc()
    
    print lr.betas
    print lr.trainMSE
    
    lr.log(testName)
    
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(lr.trainInputs[0], lr.trainOutputs, '.')
    maxVal = np.max(lr.trainInputs[0])
    minVal = np.min(lr.trainInputs[0])
    xs = np.arange(minVal, maxVal+0.01, 0.01)
    ys = xs * lr.betas[1] + lr.betas[0]
    ax.plot(xs, ys)
    title_str = "M.S.E = " + str(lr.trainMSE) + "\n"
    title_str += "  beta 0 = " + str(lr.betas[0])
    title_str += ", beta 1 = " + str(lr.betas[1])
    ax.set_title(title_str)
    #plt.show()
    plt.savefig(testName)
    