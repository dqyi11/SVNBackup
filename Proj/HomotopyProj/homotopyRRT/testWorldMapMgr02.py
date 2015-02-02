'''
Created on Jan 23, 2015

@author: daqing_yi
'''

from WorldMapMgr import *
from InteractiveWorldMapVisualizer import *
from RRTstarPlanner import *
from HomotopyMgr import *

if __name__ == '__main__':
    
    MAP_FILE = 'map02.jpg'
    mapMgr = WorldMapMgr()
    mapMgr.load(MAP_FILE)
    
    mapMgrViz = InteractiveWorldMapVisualizer(mapMgr)
    
    mapMgrViz.initVisualize()
    
    print "CENTER GROUP " + str(len(mapMgrViz.reader.centerGroup))
    
    #tg = mapMgr.getTopologicalGraph()
    #tg.visualize(MAP_FILE)
    
    readInput = False
    while readInput==False:
        mapMgrViz.updateVisualize()
        if len(mapMgrViz.convertedTrackingPosList) > 0:
            readInput = True

    start_pos = mapMgrViz.convertedTrackingPosList[0]
    end_pos = mapMgrViz.convertedTrackingPosList[len(mapMgrViz.convertedTrackingPosList)-1]
    
    def calcDist(currentPos, referencePos):
        dist = 0.0
        if referencePos==None:
            return dist
        dist = np.sqrt((currentPos[0]-referencePos[0])**2+(currentPos[1]-referencePos[1])**2)
        return dist   


    refStrPath, dumbList = mapMgrViz.reader.readPath(mapMgrViz.convertedTrackingPosList)
    homoMgr = HomotopyMgr(mapMgrViz.world_map, mapMgrViz.reader)
    homoMgr.init(refStrPath)
    planner = RRTstarPlanner([mapMgr.width, mapMgr.height], 10, calcDist, MAP_FILE) 
    
    for subseg in mapMgr.subsegments:
        planner.rrts_viz.refLines.append([subseg.line_seg.coords[0], subseg.line_seg.coords[1]])

    path = planner.findPath(start_pos, end_pos, 6000,homoMgr)
    print path
    
    import pygame.image
    pygame.image.save(planner.rrts_viz.screen, 'RRTstar02.png')
    
    while True:
        planner.rrts_viz.update()