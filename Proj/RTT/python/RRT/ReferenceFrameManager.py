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
                
        for obs in self.worldmap.obstacles:
            for a in obs.alpha_lines:
                print a.rad
                self.findNeighboringSubregion(a)
                
            for b in obs.beta_lines:
                print b.rad
                self.findNeighboringSubregion(b)
        
        g.visualize("world")
        
    def findNeighboringSubregion(self, line):
        
        # find neighboring subregion
        rad = line.rad
        
        region_s_rads = []
        region_e_rads = []
        for r in self.regions:
            region_s_rads.append(r.line_s.rad)
            region_e_rads.append(r.line_e.rad)
        
        sorted_s_idx = np.argsort(region_s_rads)
        sorted_e_idx = np.argsort(region_e_rads)
        
        print region_s_rads
        found_s_idx = sorted_s_idx[0]
        i = 0
        while rad > region_s_rads[sorted_s_idx[i]] and i < len(region_s_rads):
            if i==len(region_s_rads)-1:
                found_s_idx = sorted_s_idx[i]
            else:
                if rad < region_s_rads[sorted_s_idx[i+1]]:
                    if np.abs(rad-region_s_rads[sorted_s_idx[i]]) < np.abs(rad-region_s_rads[sorted_s_idx[i+1]]):
                        found_s_idx = sorted_s_idx[i]
                    else:
                        found_s_idx = sorted_s_idx[i+1]
            i+=1
        print "Found S rad " + str(region_s_rads[found_s_idx])
        
        print region_e_rads
        found_e_idx = sorted_e_idx[0]
        i = 0
        while rad > region_e_rads[sorted_e_idx[i]] and i < len(region_e_rads):
            if i==len(region_e_rads)-1:
                found_e_idx = sorted_e_idx[i]
            else:
                if rad < region_e_rads[sorted_e_idx[i+1]]:
                    if np.abs(rad-region_e_rads[sorted_e_idx[i]]) < np.abs(rad-region_e_rads[sorted_e_idx[i+1]]):
                        found_e_idx = sorted_e_idx[i]
                    else:
                        found_e_idx = sorted_e_idx[i+1]
            i+=1
        print "Found E rad " + str(region_e_rads[found_e_idx])
                    
        
        
            
        
        
        