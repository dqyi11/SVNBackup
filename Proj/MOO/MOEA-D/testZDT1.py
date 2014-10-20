'''
Created on Oct 18, 2014

@author: daqing_yi
'''

from moead import *

def zdt1_func(position):
    vals = np.zeros(2, np.float)
    return vals

if __name__ == '__main__':

    position_range = []
    for i in range(30):
        position_range.append([0.0, 1.0])
        
    moead = MOEAD(2, 30, zdt1_func)
    moead.initPopulation(position_range)
    
    moead.run(50)