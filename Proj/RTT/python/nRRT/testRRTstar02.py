'''
Created on Jan 4, 2015

@author: daqing_yi
'''

from RRTstar import *
from RRTVisualizer import *

if __name__ == '__main__':
    
    MAP_FILE = './map.png'
    rrts = RRTstar([640, 400], 10)
    rrts.loadMap(MAP_FILE)
    
    rrts_viz = RRTVisualizer(rrts)
    
    rrts.init([40,40], [500, 300])
    #rrt.init([40,40], [200, 200])
    
    for i in range(2000):
        rrts.extend()
        rrts_viz.update()