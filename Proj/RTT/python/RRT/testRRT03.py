'''
Created on Nov 12, 2014

@author: daqing_yi
'''

from RRTstar import *
from RRTVisualizer import *

if __name__ == '__main__':
    
    rrt = RRTstar([400, 400], 10)
    rrt_viz = RRTVisualizer(rrt)
    
    rrt.init([40,40], [300, 300])
    
    for i in range(1000):
        print i
        rrt.expand()
        rrt_viz.update()