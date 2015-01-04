'''
Created on Jan 3, 2015

@author: daqing_yi
'''

from RRG import *
from RRTVisualizer import *

if __name__ == '__main__':
    
    MAP_FILE = './map.png'
    rrg = RRG([640, 400], 10)
    rrg.loadMap(MAP_FILE)
    
    rrg_viz = RRTVisualizer(rrg)
    
    rrg.init([40,40], [500, 300])
    #rrt.init([40,40], [200, 200])
    
    for i in range(2000):
        rrg.extend()
        rrg_viz.update()