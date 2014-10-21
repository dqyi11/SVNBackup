'''
Created on 2014-10-18

@author: Walter
'''

import matplotlib.pyplot as plt;

def VisualizeParetoFront(solutionFitness, paretoFront, title=""):
    
    fig = plt.figure()     
    ax = fig.add_subplot(111)
    ax.plot(paretoFront[0], paretoFront[1],'r')
    ax.plot(solutionFitness[0], solutionFitness[1],'b.')
    ax.set_title(title)
    plt.show()