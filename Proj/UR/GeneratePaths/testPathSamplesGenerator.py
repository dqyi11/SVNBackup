'''
Created on Jul 31, 2015

@author: daqing_yi
'''

from World import *
from WorldViz import *
from PathSamplesGenerator import *

if __name__ == '__main__':
    
    world01 = World()
    world01.fromXML("world01.xml")
    
    worldViz01 = WorldViz(world01)
    cont = True
    while cont==True:
        cont = worldViz01.update()
    worldViz01.close()
    
    MAP = "map01.png"
    MAX_RUN_NUM = 5000
    SEGMENT = 10
    gnr01 = PathSamplesGenerator(worldViz01, MAP, MAX_RUN_NUM, SEGMENT) 
    gnr01.run(20)
    