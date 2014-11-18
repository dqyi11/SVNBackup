'''
Created on Nov 17, 2014

@author: daqing_yi
'''

from Shapes import *
from TopologicalGraph import *

class ReferenceFrameManager(object):
    
    def __init__(self, worldmap):
        self.frames = []
        self.worldmap = worldmap
        for obs in self.worldmap.obstacles:
            self.frames.append({"id":obs.idx, "type":"A", "rad":obs.alpha_rad})
            self.frames.append({"id":obs.idx, "type":"B", "rad":obs.beta_rad})
        self.frames = sorted(self.frames, key=lambda k: k['rad']) 
        
        self.frame_num = len(self.frames)
        self.regions = []
        for i in range(self.frame_num):
            if i < self.frame_num - 1:
                frame_s = self.frames[i]
                frame_e = self.frames[i+1]
            else:
                frame_s = self.frames[i]
                frame_e = self.frames[0]
                
            if frame_s["type"] == "A":
                point_s = self.worldmap.obstacles[frame_s["id"]].alpha_end
            else:
                point_s = self.worldmap.obstacles[frame_s["id"]].beta_end
            if frame_e["type"] == "A":
                point_e = self.worldmap.obstacles[frame_e["id"]].alpha_end
            else:
                point_e = self.worldmap.obstacles[frame_e["id"]].beta_end
                
            region = Region(point_s, point_e, self.worldmap, i)
            self.regions.append(region)
            
        g = TopologicalGraph()
        for i in range(self.frame_num):
            r = self.regions[i]
            for j in range(r.sub_region_num):
                g.addNode(str(i)+"-"+str(j))
        g.visualize("world")
            
        
        
        