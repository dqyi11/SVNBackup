'''
Created on Jan 23, 2015

@author: daqing_yi
'''

from WorldMapMgr import *

if __name__ == '__main__':
    
    MAP_FILE = 'map01.jpg'
    mapMgr = WorldMapMgr()
    mapMgr.load(MAP_FILE)
    
    mapMgr.initVisualize()
    mapMgr.init()
    mapMgr.updateVisualize()
    mapMgr.initSegments()
    mapMgr.updateVisualize()
    mapMgr.process()
    
    
    while True:
        mapMgr.updateVisualize()