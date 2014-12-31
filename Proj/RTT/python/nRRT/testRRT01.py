'''
Created on Nov 3, 2014

@author: daqing_yi
'''

from RRT import *
from RRTVisualizer import *

if __name__ == '__main__':
    
    rrt = RRT([[0,400], [0,400]], 10)
    rrt_viz = RRTVisualizer(rrt)
    
    rrt.init([40,40], [300, 300])
    
    for i in range(1000):
        print i
        rrt.expand()
        rrt_viz.update()
    