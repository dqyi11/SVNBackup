'''
Created on 2014-10-18

@author: Walter
'''

import matplotlib.pyplot as plt;

def VisualizeParetoFront(solutionFitness, paretoFront, title="", enable_lim = True, x_lim=(0.0, 1.0), y_lim=(0.0, 1.0)):
    
    fig = plt.figure()     
    ax = fig.add_subplot(111)
    ax.plot(paretoFront[0], paretoFront[1],'r.')
    ax.plot(solutionFitness[0], solutionFitness[1],'b.')
    if enable_lim == True:
        ax.set_xlim(x_lim)
        ax.set_ylim(y_lim)
    ax.set_title(title)
    plt.show()