'''
Created on Jan 3, 2015

@author: daqing_yi
'''

from RRG import *
from RRTVisualizer import *

if __name__ == '__main__':
    
    rrg = RRG([600, 400], 10)

    rrg_viz = RRTVisualizer(rrg)
    
    rrg.init([40,40], [320, 300])
    
    for i in range(1000):
        #print i
        rrg.extend()
        rrg_viz.update()