'''
Created on Aug 7, 2015

@author: hcmi
'''

from World import *
import numpy as np

class WorldGenerator(object):

    def __init__(self, width, height, margin=40):
        self.width = width
        self.height = height
        self.margin = margin
        self.worlds = []
        
    def randInitWorlds(self, num, objects):
        
        for i in range(num):
            w = World()
            w.width = self.width
            w.height = self.height
            for o in objects:
                obj = Object()
                obj.name = o[0]
                obj.type = o[1]
                obj.center = []
                obj.center.append( np.random.randint(self.margin, self.width-self.margin) )
                obj.center.append( np.random.randint(self.margin, self.height-self.margin) )
                obj.randParam()
                if obj.type == "robot":
                    w.robot = obj 
                else:
                    w.objects.append(obj)
            self.worlds.append(w)
        
    def dumpXML(self, prefix):
        
        world_len = len(self.worlds)
        for i in range(world_len):
            filename = prefix+"-"+str(i)+".xml"
            self.worlds[i].dumpXML(filename)
            
            
            

        