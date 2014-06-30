'''
Created on 2014-2-14

@author: Walter
'''

import matplotlib.pyplot as plt
import numpy as np
import pickle

if __name__ == '__main__':
    
    trainDataX = pickle.load(open('trainDataX.data', 'rb'))
    trainDataC = pickle.load(open('trainDataC.data', 'rb'))
    testDataX = pickle.load(open('testDataX.data', 'rb'))
    testDataC = pickle.load(open('testDataC.data', 'rb'))
    
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(np.array(trainDataX.T), np.array(trainDataC.T), 'gx')
    ax.plot(np.array(testDataX.T), np.array(testDataC.T), 'r.')
    plt.show()