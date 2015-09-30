'''
Created on 2013-6-5

@author: Walter
'''

if __name__ == '__main__':
    
    import matplotlib.pyplot as plt
    import scipy.stats
    import numpy as np
    import math

    def meanRef(x):
        
        post_mean = 5
        post_var = 1.0/9.0
        num = len(x)
        y = np.zeros(num)
        for i in range(num):
            y[i] = (1 / (2 * math.pi * post_var)**0.5) * math.exp(- (x[i] - post_mean)**2 / (2 * post_var) )
        
        return y
    
    def varianceRef(x):
        
        post_alpha = 11
        post_beta = 2.5
        rv = scipy.stats.invgamma(post_alpha, scale=post_beta)
        num = len(x)
        y = np.zeros(num)
        for i in range(num):
            if x[i] > 0 :
                #y[i] = rv.pdf(x[i])
                y[i] = post_beta**post_alpha / math.gamma(post_alpha) * x[i]**(-post_alpha - 1) * math.exp(-post_beta / x[i])  
            else:
                y[i] = 0 
        return y
    
    x = np.arange(3.5,6.5,0.001)
    h = plt.plot(x, meanRef(x))
    plt.xlabel("Value")
    plt.ylabel("Probabiloity")
    plt.title("Prior of Mean")
    plt.show()