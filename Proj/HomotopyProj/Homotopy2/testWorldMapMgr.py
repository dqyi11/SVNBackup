'''
Created on Jan 23, 2015

@author: daqing_yi
'''

from WorldMapMgr import *
from WorldMapVisualizer import * 

if __name__ == '__main__':
    
    MAP_FILE = 'BYU_map.jpg'
    mapMgr = WorldMapMgr()
    mapMgr.load(MAP_FILE)
    
    mapMgrViz = WorldMapVisualizer(mapMgr)
    
    mapMgrViz.initVisualize()
    
    tg = mapMgr.getTopologicalGraph()
    tg.visualize(MAP_FILE)
    
    while True:
        mapMgrViz.updateVisualize()