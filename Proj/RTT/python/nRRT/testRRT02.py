'''
Created on Jan 3, 2015

@author: daqing_yi
'''

from RRT import *
from RRTVisualizer import *

if __name__ == '__main__':
    
    MAP_FILE = './map.png'
    rrt = RRT([640, 400], 10)
    rrt.loadMap(MAP_FILE)
    
    rrt_viz = RRTVisualizer(rrt)
    
    rrt.init([40,40], [500, 300])
    #rrt.init([40,40], [200, 200])
    
    for i in range(500):
        rrt.extend()
        rrt_viz.update()
    
