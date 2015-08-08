'''
Created on Aug 7, 2015

@author: hcmi
'''

from World import *
from WorldViz import *



if __name__ == '__main__':
    
    world01 = World()
    world01.fromXML("W-0.xml")

    worldViz01 = WorldViz(world01)
    cont = True
    while cont==True:
        cont = worldViz01.update()