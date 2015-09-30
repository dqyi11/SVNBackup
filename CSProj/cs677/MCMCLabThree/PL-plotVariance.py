'''
Created on 2013-6-10

@author: Walter
'''

'''
Created on 2013-6-10

@author: Walter
'''

import matplotlib.pyplot as plt
import scipy.stats

if __name__ == '__main__':
    
    variance = []
    for line in open('f4v.txt'):
        exec('variance='+line)
        
    refVariance = []
    for line in open('f0v.txt'):
        exec('refVariance='+line)
        
    plt.hist(variance, 50, normed=True, color='b', label='hyper beta of variance')
    plt.hist(refVariance, 50, normed=True, color='r', alpha=0.5, label='hard coded hyperparameter')
    plt.title("Variance Histogram")
    plt.legend()
    plt.show()
    