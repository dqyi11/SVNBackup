'''
Created on 2013-6-4

@author: Walter
'''

if __name__ == '__main__':
    import matplotlib.pyplot as plt
    import scipy.stats
    import numpy as np
    
    rv = scipy.stats.norm(0.5,0.4)
    
    Y = []
    X = np.arange(-1,1.0,0.01)
    for x in X:
        Y.append(rv.pdf(x))
        
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(X,Y)
    plt.show()
    