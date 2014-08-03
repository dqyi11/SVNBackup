'''
Created on 2014-3-23

@author: Walter
'''

if __name__ == '__main__':
    
    import numpy as np
    
    phi_g = 0.1
    phi_p = 0.5
    chi = 0.6
    
    A = [[chi, - chi * phi_g - chi * phi_g],[chi, 1 - chi * phi_g - chi * phi_g ]]
    B = [[chi * phi_g, chi * phi_p],[chi * phi_g, chi * phi_p]]
    
    print np.linalg.norm(A, ord=np.Inf)
    
    print A
    print B