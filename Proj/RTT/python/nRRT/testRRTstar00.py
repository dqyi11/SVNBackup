'''
Created on Jan 3, 2015

@author: daqing_yi
'''
from RRTstar import *
from RRTVisualizer import *

if __name__ == '__main__':
    
    rrts = RRTstar([600, 400], 10)

    rrts_viz = RRTVisualizer(rrts)
    
    rrts.init([40,40], [320, 300])
    
    for i in range(1000):
        #print i
        rrts.extend()
        rrts_viz.update()