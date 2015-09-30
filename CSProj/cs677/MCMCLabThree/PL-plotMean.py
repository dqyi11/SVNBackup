'''
Created on 2013-6-10

@author: Walter
'''

import matplotlib.pyplot as plt
import scipy.stats

if __name__ == '__main__':
    
    mean = []
    for line in open('f5m.txt'):
        exec('mean='+line)
        
    refMean = []
    for line in open('f0m.txt'):
        exec('refMean='+line)
        
    plt.hist(mean, 50, normed=True, color='b', label='four hyperparameters')
    plt.hist(refMean, 50, normed=True, color='r', alpha=0.5, label='hard coded hyperparameter')
    plt.title("Mean Histogram")
    plt.legend()
    plt.show()

