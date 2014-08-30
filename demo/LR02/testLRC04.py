from NeuralNetworkCalculator import *
import matplotlib.pyplot as plt

if __name__ == '__main__':
    
    lr = NeuralNetworkCalculator(1)
    lr.load('testData-20.csv')
    #print lr.inputs[0]
    #print lr.inputs[1]
    #print lr.inputs[2]
    #print lr.inputs[3]
    #print lr.outputs
    #lr.calc()
    
    lr.runCnt = 100
    lr.calcByGA(200, [-5.0, 5.0])
    #lr.calcByPSO(200, [-5.0, 5.0])
    
    print lr.betas
    #print lr.mle
    
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(lr.inputs[0], lr.outputs, '.')
    maxVal = np.max(lr.inputs[0])
    minVal = np.min(lr.inputs[0])
    xs = np.arange(minVal, maxVal+0.01, 0.01)
    ys = []
    for x in xs:
        ys.append(lr.nn.calcFunc(lr.betas, [x]))
    ax.plot(xs, ys)
    ax.set_title("M.L.E = " + str(lr.mle))
    plt.show()
    