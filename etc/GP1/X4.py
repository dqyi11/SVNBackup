'''
Created on 2014-2-17

@author: Walter
'''

import numpy as np
import matplotlib.pyplot as plt

if __name__ == '__main__':
    
    X = np.random.random(10)
    Y = np.random.random(10)
    
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(X,Y,'sb')
    plt.show()