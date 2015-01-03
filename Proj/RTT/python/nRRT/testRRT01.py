'''
Created on Nov 3, 2014

@author: daqing_yi
'''

from RRT import *
from RRTVisualizer import *

if __name__ == '__main__':
    
    MAP_FILE = './lab-map-inferior.png'
    MAP_FILE = './lab-map-scaled.png'
    rrt = RRT([444, 989], 10)
    rrt.loadMap(MAP_FILE)
    
    rrt_viz = RRTVisualizer(rrt)
    
    rrt.init([40,40], [320, 300])
    #rrt.init([40,40], [200, 200])
    
    for i in range(2000):
        rrt.extend()
        rrt_viz.update()
    