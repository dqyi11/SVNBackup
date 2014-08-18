'''
Created on Aug 14, 2014

@author: daqing_yi
'''

import numpy as np
import matplotlib.pyplot as plt

if __name__ == '__main__':
    
    xs = np.arange(0.0,1.01,0.01)
    
    y2 = []
    y4 = []
    y8 = []
    y16 = []
    y32 = []
    y64 = []
    for x in xs:
        y2.append( 1 - (1-x)**2 )
        y4.append( 1 - (1-x)**4 )
        y8.append( 1 - (1-x)**8 )
        y16.append( 1 - (1-x)**16 )
        y32.append( 1 - (1-x)**32 )
        y64.append( 1 - (1-x)**64 )
    
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(xs,xs,xs,y2,xs,y4,xs,y8,xs,y16,xs,y32,xs,y64)
    
    ax.legend(['N=1','N=2','N=4','N=8','N=16','N=32','N=64'],loc=4)
    
    plt.show()