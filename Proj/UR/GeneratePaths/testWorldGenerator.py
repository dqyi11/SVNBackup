'''
Created on Aug 7, 2015

@author: hcmi
'''

from WorldGenerator import *

if __name__ == '__main__':
    
    w = 800
    h = 600
    worldGnr = WorldGenerator(w, h)
    
    objects = []
    objects.append(["R", "robot"])
    objects.append(["P", "person"])
    objects.append(["C", "car"])
    objects.append(["T", "tree"])
    objects.append(["B1", "building"])
    objects.append(["B2", "building"])
    
    worldGnr.randInitWorlds(1, objects)
    
    worldGnr.dumpXML("W")