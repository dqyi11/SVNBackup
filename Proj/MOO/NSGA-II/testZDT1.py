'''
Created on Oct 18, 2014

@author: daqing_yi
'''

def zdt1_func(position):
    vals = np.zeros(2, np.float)
    return vals

from nsga2 import *

if __name__ == '__main__':

    position_range = []
    for i in range(30):
        position_range.append([0.0, 1.0])
        
    nsga2 = NSGAII(2, 30, zdt1_func)
    nsga2.initPopulation(20, position_range)
    
    nsga2.run(50)
    