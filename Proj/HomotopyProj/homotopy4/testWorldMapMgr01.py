'''
Created on Jan 23, 2015

@author: daqing_yi
'''

from WorldMapMgr import *
from InteractiveWorldMapVisualizer import *

if __name__ == '__main__':
    
    MAP_FILE = 'map01.jpg'
    mapMgr = WorldMapMgr()
    mapMgr.load(MAP_FILE)
    
    mapMgrViz = InteractiveWorldMapVisualizer(mapMgr)
    
    mapMgrViz.initVisualize()
    
    tg = mapMgr.initTopologicalGraph()
    tg.visualize(MAP_FILE)
    
    while mapMgrViz.start ==None and mapMgrViz.end ==None:
        mapMgrViz.updateVisualize()
   
    print "START: " + mapMgrViz.start.getName() 
    print "END: " + mapMgrViz.end.getName() 
    
    allpaths = tg.findAllPathsByBFS(mapMgrViz.start.getName(), mapMgrViz.end.getName())
    
    for path in allpaths:
        print str(path)
        
    while True:
        mapMgrViz.updateVisualize()
            