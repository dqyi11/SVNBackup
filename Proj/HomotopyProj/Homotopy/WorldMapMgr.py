'''
Created on Jan 23, 2015

@author: daqing_yi
'''

import cv2

from ObstacleMgr import *
from RegionMgr import *
import sympy.geometry as symgeo
import sympy.core as symcore
import shapely.geometry as shpgeo
import numpy 

import pygame, sys
from pygame.locals import *

OBSTACLE_COLOR = (122,122,122)
ALPHA_COLOR = (0,0,255)
BETA_COLOR = (0,255,0)
ALPHA_SELF_COLOR = (153,204,255)
ALPHA_OTHER_COLOR = (0, 76, 153)
BETA_SELF_COLOR = (153,255,204)
BETA_OTHER_COLOR = (76,153,0)
CENTER_POINT_COLOR = (255,0,0)
OBS_BK_COLOR = (124,252,0)



class WorldMapMgr(object):

    def __init__(self):
        self.obstacles = []
    
    def load(self, mapfile):
        self.mapfile = mapfile
        self.mapImgData = cv2.imread(mapfile)
        self.height = self.mapImgData.shape[0]
        self.width = self.mapImgData.shape[1]
        self.mapImgGrayData = cv2.cvtColor(self.mapImgData, cv2.COLOR_BGR2GRAY)

        ret, thresh = cv2.threshold(255-self.mapImgGrayData, 127, 255, 0)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        for cont in contours:
            obs = ObstacleMgr(cont, len(self.obstacles))
            self.obstacles.append(obs)
            
        self.init()
        
    def init(self):
        # select random point for each obstacle
        for obs in self.obstacles:
            obs.bk = obs.samplePosition()
            
        self.obsBkPairLines = []
        for i in range(len(self.obstacles)):
            for j in range(i+1, len(self.obstacles)):
                bk1 = self.obstacles[i].bk
                bk2 = self.obstacles[j].bk
                pline = symgeo.Line(symgeo.Point(bk1[0], bk1[1]),symgeo.Point(bk2[0], bk2[1]))
                self.obsBkPairLines.append(pline)
                
        
        # select central point c
        self.centralPoint = (int(self.width/2), int(self.height/2))
        foundCP = False
        while foundCP == False:
            if self.isInObsBkLinePair(self.centralPoint)==False:
                foundCP = True
            else:
                self.centralPoint[0] = np.random.randint(0, self.width)
                self.centralPoint[1] = np.random.randint(0, self.height)
                
        # init four boundary line
        self.boundary_lines = []
        self.x_axis = symgeo.Line(symgeo.Point(0,0), symgeo.Point(self.width-1,0))
        self.y_axis = symgeo.Line(symgeo.Point(0,0), symgeo.Point(0, self.height-1))
        self.x_high = symgeo.Line(symgeo.Point(0,self.height-1), symgeo.Point(self.width-1, self.height-1))
        self.y_high = symgeo.Line(symgeo.Point(self.width-1, 0), symgeo.Point(self.width-1, self.height-1))
        self.boundary_lines.append(self.x_axis)
        self.boundary_lines.append(self.y_axis)
        self.boundary_lines.append(self.x_high)
        self.boundary_lines.append(self.y_high)
        
        self.ray_info_list = []
        
        # init alpah and beta segments
        for obs in self.obstacles:
            obs.alpha_ray = symgeo.Ray(symgeo.Point(obs.bk[0], obs.bk[1]), symgeo.Point(self.centralPoint[0],self.centralPoint[1]))
            obs.beta_ray = symgeo.Ray(symgeo.Point(obs.bk[0], obs.bk[1]), symgeo.Point(2*obs.bk[0]-self.centralPoint[0], 2*obs.bk[1]-self.centralPoint[1])) 
            
            a_pt = self.findIntersectionWithBoundary(obs.alpha_ray)
            #print str(obs.alpha_ray) + " --> " + str(a_pt)
            b_pt = self.findIntersectionWithBoundary(obs.beta_ray)
            #print str(obs.beta_ray) + " --> " + str(b_pt)
            
            obs.alpha_seg = None
            obs.beta_seg = None
            if a_pt != None:
                obs.alpha_seg = shpgeo.LineString([obs.bk, (int(a_pt.x), int(a_pt.y))])
            if b_pt != None:
                obs.beta_seg = shpgeo.LineString([obs.bk, (int(b_pt.x), int(b_pt.y))])
                
            alpha_ray_rad = numpy.arctan(float(obs.alpha_ray.slope))
            if obs.alpha_ray.p1.x > obs.alpha_ray.p2.x:
                alpha_ray_rad += numpy.pi
            if alpha_ray_rad < 0:
                alpha_ray_rad += 2*numpy.pi
            alpha_ray_info = (obs.idx, 'A', alpha_ray_rad)
            
            beta_ray_rad = numpy.arctan(float(obs.beta_ray.slope))
            if obs.beta_ray.p1.x > obs.beta_ray.p2.x:
                beta_ray_rad += numpy.pi
            if beta_ray_rad < 0:
                beta_ray_rad += 2*numpy.pi                
            beta_ray_info = (obs.idx, 'B', beta_ray_rad)
            #print "ALPHA RAY SLOPE " + str(alpha_ray_info)
            #print "BETA RAY SLOPE " + str(beta_ray_info)
            self.ray_info_list.append(alpha_ray_info)
            self.ray_info_list.append(beta_ray_info)
                
        self.ray_info_list.sort(key=lambda x: x[2], reverse=False)
        #print self.ray_info_list
                
        for obs in self.obstacles:
            
            alpha_self_intsecs = obs.alpha_seg.intersection(obs.polygon)
            #print "ALPHA: " + str(obs.alpha_seg) + " + " + str(obs.idx) + " = " + str(alpha_self_intsecs)
            if alpha_self_intsecs.is_empty == False:
                for c in list(alpha_self_intsecs.coords):
                    obs.alpha_self_intsecs.append((int(c[0]), int(c[1])))   
            
            beta_self_intsecs = obs.beta_seg.intersection(obs.polygon)
            #print "BETA: " + str(obs.beta_seg) + " + " + str(obs.idx) + " = " + str(beta_self_intsecs)
            if beta_self_intsecs.is_empty == False:
                for c in list(beta_self_intsecs.coords):
                    obs.beta_self_intsecs.append((int(c[0]), int(c[1])))   
                    
            for other_obs in self.obstacles:
                if obs != other_obs:
                    # check alpha seg with obstacles
                    alpha_intsecs = obs.alpha_seg.intersection(other_obs.polygon)
                    #print "ALPHA: " + str(obs.alpha_seg) + " + " + str(other_obs.idx) + " = " + str(alpha_intsecs)
                    if alpha_intsecs.is_empty == False:
                        for c in list(alpha_intsecs.coords):
                            cpos = (int(c[0]), int(c[1]))
                            obs.alpha_obs_intsecs.append(cpos)
                            obs.alpha_obs_intsecs_info.append((other_obs.idx, 'A', cpos))              
                    
                    # check beta seg with obstacles
                    beta_intsecs = obs.beta_seg.intersection(other_obs.polygon)
                    #print "BETA: " + str(obs.beta_seg) + " + " + str(other_obs.idx) + " = " + str(beta_intsecs)
                    if beta_intsecs.is_empty == False:
                        for c in list(beta_intsecs.coords):
                            cpos = (int(c[0]), int(c[1]))
                            obs.beta_obs_intsecs.append(cpos)
                            obs.beta_obs_intsecs_info.append((other_obs.idx, 'B', cpos))  
                            
            #print str(obs.alpha_obs_intsecs)
            #print str(obs.beta_obs_intsecs)
            #print obs.alpha_obs_intsecs_info
            #print obs.beta_obs_intsecs_info
            
    def process(self):
        
        # split the region
        for i in range(len(self.ray_info_list)-1):
            ray1_info = self.ray_info_list[i]
            ray2_info = self.ray_info_list[i+1]
            region = RegionMgr(ray1_info, ray2_info, i, self)
            self.regions.append(region)
         
            
        
    def isInObsBkLinePair(self, pos):
        
        pPos = symgeo.Point(pos[0], pos[1])
        for bkline in self.obsBkPairLines:
            if bkline.contains(pPos):
                return True
        return False    
        
    def visualize(self):
        pygame.init()
        self.screen = pygame.display.set_mode((self.width,self.height))
        pygame.display.set_caption(self.mapfile)
        self.screen.fill((255,255,255))
        
        for obs in self.obstacles:
            xs, ys = obs.polygon.exterior.coords.xy
            for i in range(len(xs)-1):
                pygame.draw.line(self.screen, OBSTACLE_COLOR, (xs[i], ys[i]), (xs[i+1], ys[i+1]), 2)
            if obs.alpha_seg != None:
                pygame.draw.line(self.screen, ALPHA_COLOR, obs.alpha_seg.coords[0], obs.alpha_seg.coords[1], 2)
            if obs.beta_seg != None:
                pygame.draw.line(self.screen, BETA_COLOR, obs.beta_seg.coords[0], obs.beta_seg.coords[1], 2)
            for ac in obs.alpha_self_intsecs:
                pygame.draw.circle(self.screen, ALPHA_SELF_COLOR, ac, 3)
            for bc in obs.beta_self_intsecs:
                pygame.draw.circle(self.screen, BETA_SELF_COLOR, bc, 3)
            for ac in obs.alpha_obs_intsecs:
                pygame.draw.circle(self.screen, ALPHA_OTHER_COLOR, ac, 3)
            for bc in obs.beta_obs_intsecs:
                pygame.draw.circle(self.screen, BETA_OTHER_COLOR, bc, 3)
            pygame.draw.circle(self.screen, OBS_BK_COLOR, obs.bk, 3)
            
        pygame.draw.circle(self.screen, CENTER_POINT_COLOR, self.centralPoint, 3)     
            
        while True:
            for event in pygame.event.get():
                if event.type == QUIT:
                    pygame.quit()
                    sys.exit()
            pygame.display.update()
            
            
    def findIntersectionWithBoundary(self, a_ray):
        for bl in self.boundary_lines:
            intsecs = symgeo.intersection(bl, a_ray)
            #print str(a_ray) + " + " + str(bl) + " = " + str(intsecs)
            if len(intsecs) > 0:
                intsec = intsecs[0]
                if intsec.x >=0 and intsec.x <self.width and intsec.y >=0 and intsec.y <self.height:
                    return intsec
        return None

