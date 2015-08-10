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
    
    for i in range(WORLD_NUM):
        
        NAME = WORLD_PREFIX + "-" +str(i)
        DIR_NAME = "./" + NAME
        
        if not os.path.exists(DIR_NAME):
            os.makedirs(DIR_NAME)
            
        world_file = WORLD_PREFIX + "-" + str(i) + ".xml"
        worldGnr.worlds[i].dumpXML(DIR_NAME + "/" + world_file)
        
        wrd = worldGnr.worlds[i]
        wrd.initGoal()
        
        wViz = WorldViz(wrd)

        wGnr = PathSamplesGenerator(wViz, MAP, MAX_RUN_NUM, SEGMENT, DIR_NAME+"/") 
        wGnr.run(PATH_NUM, NAME)