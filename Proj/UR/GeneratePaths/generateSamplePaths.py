'''
Created on Aug 7, 2015

@author: hcmi
'''

from WorldGenerator import *
from World import *
from WorldViz import *
from PathSamplesGenerator import *

if __name__ == '__main__':
    
    WORLD_NUM = 1
    PATH_NUM = 2
    
    MAP = "map01.png"
    MAX_RUN_NUM = 5000
    SEGMENT = 10
    
    WORLD_WIDTH = 800
    WORLD_HEIGHT = 600
    worldGnr = WorldGenerator(WORLD_WIDTH, WORLD_HEIGHT)
    
    objects = []
    objects.append(["R", "robot"])
    objects.append(["P", "person"])
    objects.append(["C", "car"])
    objects.append(["T", "tree"])
    objects.append(["B1", "building"])
    objects.append(["B2", "building"])
    
    worldGnr.randInitWorlds(WORLD_NUM, objects)
    
    WORLD_PREFIX = "W"
    
    worldGnr.dumpXML(WORLD_PREFIX)
    
    for i in range(WORLD_NUM):
        
        filename = WORLD_PREFIX + "-" +str(i) + ".xml"
        world01 = World()
        world01.fromXML(filename)
        world01.initGoal()
        
        worldViz01 = WorldViz(world01)


        gnr01 = PathSamplesGenerator(worldViz01, MAP, MAX_RUN_NUM, SEGMENT) 
        gnr01.run(PATH_NUM)