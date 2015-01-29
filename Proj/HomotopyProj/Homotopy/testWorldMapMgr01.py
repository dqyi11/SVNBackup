'''
Created on Jan 23, 2015

@author: daqing_yi
'''

from WorldMapMgr import *
from WorldMapVisualizer import *

if __name__ == '__main__':
    
    MAP_FILE = 'map01.jpg'
    mapMgr = WorldMapMgr()
    mapMgr.load(MAP_FILE)
    
    mapMgrViz = WorldMapVisualizer(mapMgr)
    
    mapMgrViz.initVisualize()
    mapMgr.init()
    mapMgrViz.updateVisualize()
    mapMgr.initSegments()
    mapMgrViz.updateVisualize()
    mapMgr.process()
    
    
    while True:
        mapMgrViz.updateVisualize()