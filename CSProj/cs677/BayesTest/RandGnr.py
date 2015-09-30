'''
Created on 2013-5-13

@author: Walter
'''
if __name__ == '__main__':
    
    import numpy as np
    import matplotlib.pyplot as plt
    
    n = 100000
    X = np.random.random(n)
    Y = -0.5 + np.sqrt(2 * (X + 0.125))    
    plt.hist(Y, 100, normed=1, histtype='step')
        
    refX = np.arange(0, 1, 0.01)
    refY = refX + 0.5
    
    plt.plot(refX, refY)  
    plt.show()