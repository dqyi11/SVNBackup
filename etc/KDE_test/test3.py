'''
Created on 2014-1-20

@author: Walter
'''

if __name__ == '__main__':
    
    from scipy import stats
    import numpy as np
    
    m1 = np.random.normal(size=2)
    m1[0] = -3.26284232
    m1[1] = -3.26284232
    print m1
    values = np.vstack([m1])
    kernel = stats.gaussian_kde(values)
    
    print kernel.resample(1)