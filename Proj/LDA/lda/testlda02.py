'''
Created on Dec 6, 2014

@author: daqing_yi
'''

import numpy as np

if __name__ == '__main__':
    
    citeseer_content = np.loadtxt('citeseer.content', delimiter='\t', dtype=str)
    citeseer_cites = np.genfromtxt('citeseer.cites', delimiter='\t', dtype=str)
    
    print citeseer_content.shape
    print citeseer_cites.shape
    
    print citeseer_content[0]