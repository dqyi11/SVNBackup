'''
Created on 2013-6-5

@author: Walter
'''

if __name__ == '__main__':
    
    import scipy.stats as sciST
    
    rv = sciST.invgamma(3.0, loc=0.0, scale=7.0)
    
    print rv.mean()
    
    rv2 = sciST.gamma(3.0, loc=0.0, scale=7.0)
    print rv2.mean()