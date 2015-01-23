'''
Created on Nov 3, 2014

@author: daqing_yi
'''

from WorldMap import *

if __name__ == '__main__':
    
    world_file = 'map01.png'
    world = WorldMap()
    world.load(world_file)
    
    #world.dump(world_file+".csv")
    #for r in world.rf_mgr.regions:
    #    r.visualize()
    world.visualize()