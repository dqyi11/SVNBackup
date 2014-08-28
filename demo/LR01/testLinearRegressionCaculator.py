from LinearRegressionCalculator import *
import matplotlib.pyplot as plt

if __name__ == '__main__':
    
    lr = LinearRegressionCalculator()
    lr.load('testData-200.csv')
    lr.calc()
    
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(lr.inputs, lr.outputs, '.')
    maxVal = np.max(lr.inputs)
    minVal = np.min(lr.inputs)
    xs = np.arange(minVal, maxVal+0.01, 0.01)
    ys = xs * lr.beta1 + lr.beta0
    ax.plot(xs, ys)
    ax.set_title("M.L.E = " + str(lr.mle))
    plt.show()