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
        
    def randInitWorlds(self, num, objects, grid=(3,3)):
        
        for i in range(num):
            w = World()
            w.width = self.width
            w.height = self.height
            for o in objects:
                obj = Object()
                obj.name = o[0]
                obj.type = o[1]
                obj.center = []
                
                x, y = self.randPos(grid)       
                obj.center.append(x)
                obj.center.append(y)         
                #obj.center.append( np.random.randint(self.margin, self.width-self.margin) )
                #obj.center.append( np.random.randint(self.margin, self.height-self.margin) )
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
            
    def randPos(self, grid):
        w = grid[0]
        h = grid[1]
        idx = np.random.randint(0, w*h)
        wi = idx % w
        hi = int( (idx - wi) / w)
        ws = int( (self.width - 2*self.margin)/w )
        hs = int( (self.height - 2*self.margin)/h )
        x = int( np.random.randint( self.margin+wi*ws , self.margin+(wi+1)*ws ) )
        y = int( np.random.randint( self.margin+hi*hs , self.margin+(hi+1)*hs ) )    
        return x, y
            

        