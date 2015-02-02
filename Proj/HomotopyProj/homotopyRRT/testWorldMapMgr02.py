'''
Created on Jan 23, 2015

@author: daqing_yi
'''

from WorldMapMgr import *
from InteractiveWorldMapVisualizer import *

if __name__ == '__main__':
    
    MAP_FILE = 'map02.jpg'
    mapMgr = WorldMapMgr()
    mapMgr.load(MAP_FILE)
    
    mapMgrViz = InteractiveWorldMapVisualizer(mapMgr)
    
    mapMgrViz.initVisualize()
    
    tg = mapMgr.getTopologicalGraph()
    #tg.visualize(MAP_FILE)
    
    while True:
        mapMgrViz.updateVisualize()