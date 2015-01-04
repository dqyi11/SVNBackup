'''
Created on Jan 3, 2015

@author: daqing_yi
'''

from RRG import *
from RRTVisualizer import *

if __name__ == '__main__':
    
    MAP_FILE = './lab-map-inferior.png'
    MAP_FILE = './lab-map-scaled.png'
    rrg = RRG([444, 989], 10)
    rrg.loadMap(MAP_FILE)
    
    rrg_viz = RRTVisualizer(rrg)
    
    rrg.init([40,40], [320, 300])
    #rrt.init([40,40], [200, 200])
    
    for i in range(2000):
        rrg.extend()
        rrg_viz.update()
    