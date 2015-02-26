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
    
    while mapMgrViz.start ==None and mapMgrViz.end ==None:
        mapMgrViz.updateVisualize()
        
    tg = mapMgr.initTopologicalGraph()
    tg.visualize(MAP_FILE)
   
    print "START: " + mapMgrViz.start.getName() 
    print "END: " + mapMgrViz.end.getName() 
    
    mapMgrViz.allHomotopyClasses = tg.findAllPathsByBFS(mapMgrViz.start.getName(), mapMgrViz.end.getName())
    
    for path in mapMgrViz.allHomotopyClasses:
        print str(path)
        
    while True:
        mapMgrViz.updateVisualize()