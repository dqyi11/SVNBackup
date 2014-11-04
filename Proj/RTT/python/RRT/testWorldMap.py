'''
Created on Nov 3, 2014

@author: daqing_yi
'''

from WorldMap import *

world_file = 'map01.png'
world = WorldMap()
world.load(world_file)

#world.dump(world_file+".csv")
world.visualize()