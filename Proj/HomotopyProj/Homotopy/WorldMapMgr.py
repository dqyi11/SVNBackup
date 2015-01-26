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
SUBSEGMENT_COLOR = (255,255,0,100)
SUBREGION_BORDER_COLOR = (255,0,0,100)



class WorldMapMgr(object):

    def __init__(self):
        self.obstacles = []
        self.regions = []
        self.region_colors = []
        
        self.subsegments = []
    
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
        self.process()
        
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
        self.boundary_lines.append(self.y_high)
        self.boundary_lines.append(self.x_high)
        self.boundary_lines.append(self.y_axis)
        
        # init lines from center point to four corners
        self.center_corner_lines_info = []
        self.center_corner_lines_info.append( [ (0,0), numpy.arctan2(float(-self.centralPoint[1]), float(-self.centralPoint[0])) ] )
        self.center_corner_lines_info.append( [ (0,self.height), numpy.arctan2(float(self.height-self.centralPoint[1]), float(-self.centralPoint[0])) ] )
        self.center_corner_lines_info.append( [ (self.width,self.height), numpy.arctan2(float(self.height-self.centralPoint[1]), float(self.width-self.centralPoint[0])) ] )
        self.center_corner_lines_info.append( [ (self.width,0), numpy.arctan2(float(-self.centralPoint[1]), float(self.width-self.centralPoint[0])) ] )
        for ccl_info in self.center_corner_lines_info:
            if ccl_info[1] < 0:
                ccl_info[1] += 2*numpy.pi
        self.center_corner_lines_info.sort( key=lambda x: x[1], reverse=False )
        #print "CENTER CORNER LINES "
        #print self.center_corner_lines_info
        
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
                alpha_seg = shpgeo.LineString([obs.bk, (int(a_pt.x), int(a_pt.y))])
                obs.alpha_seg = LineSegmentMgr(alpha_seg, 'A', obs)
            if b_pt != None:
                beta_seg = shpgeo.LineString([obs.bk, (int(b_pt.x), int(b_pt.y))])
                obs.beta_seg = LineSegmentMgr(beta_seg, 'B', obs)
                
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
              
            obs.alpha_seg_info = ( (int(a_pt.x), int(a_pt.y)), alpha_ray_rad )
            obs.beta_seg_info = ( (int(b_pt.x), int(b_pt.y)), beta_ray_rad )  
              
                
        self.ray_info_list.sort(key=lambda x: x[2], reverse=False)
        #print self.ray_info_list
                
        for obs in self.obstacles:
            
            for obs_ref in self.obstacles:
                
                # check alpha seg with obstacles
                if obs.alpha_seg.line_seg.intersects(obs_ref.polygon):
                    alpha_intsecs = obs.alpha_seg.line_seg.intersection(obs_ref.polygon)
                    #alpha_intsecs = obs_ref.polygon.intersection(obs.alpha_seg.line_seg)
                    #print "ALPHA: " + str(obs.alpha_seg) + " + " + str(other_obs.idx) + " = " + str(alpha_intsecs)
                    if alpha_intsecs.is_empty == False:
                        for c in list(alpha_intsecs.coords):
                            cpos = (int(c[0]), int(c[1]))
                            cdist = numpy.sqrt((obs.bk[0]-cpos[0])**2+(obs.bk[1]-cpos[1])**2)
                            obs.alpha_obs_intsecs.append(cpos)
                            obs.alpha_obs_intsecs_info.append((obs_ref.idx, 'A', cpos, cdist))              
                
                if obs.beta_seg.line_seg.intersects(obs_ref.polygon):
                    # check beta seg with obstacles
                    beta_intsecs = obs.beta_seg.line_seg.intersection(obs_ref.polygon)
                    #print "BETA: " + str(obs.beta_seg) + " + " + str(other_obs.idx) + " = " + str(beta_intsecs)
                    if beta_intsecs.is_empty == False:
                        for c in list(beta_intsecs.coords):
                            cpos = (int(c[0]), int(c[1]))
                            cdist = numpy.sqrt((obs.bk[0]-cpos[0])**2+(obs.bk[1]-cpos[1])**2)
                            obs.beta_obs_intsecs.append(cpos)
                            obs.beta_obs_intsecs_info.append((obs_ref.idx, 'B', cpos, cdist))
            
            obs.alpha_obs_intsecs_info.sort(key=lambda x: x[3], reverse=False)
            obs.beta_obs_intsecs_info.sort(key=lambda x: x[3], reverse=False)
            obs.alpha_seg.load(obs.alpha_obs_intsecs_info)
            obs.beta_seg.load(obs.beta_obs_intsecs_info)
            
        for obs in self.obstacles:
            for linSeg in obs.alpha_seg.sub_segs:
                self.subsegments.append(linSeg)
            for linSeg in obs.beta_seg.sub_segs:
                self.subsegments.append(linSeg)
            
            
    def process(self):
        
        # split the region
        self.regions = []
        self.region_colors = []
        for i in range(len(self.ray_info_list)-1):
            ray1_info = self.ray_info_list[i]
            ray2_info = self.ray_info_list[i+1]
            region = RegionMgr(ray1_info, ray2_info, i, self)
            self.regions.append(region)
            rndVal = numpy.random.randint(0, 256, 3)
            self.region_colors.append((rndVal[0], rndVal[1], rndVal[2]))
        ray1_info = self.ray_info_list[len(self.ray_info_list)-1]
        ray2_info = self.ray_info_list[0]
        region = RegionMgr(ray1_info, ray2_info, len(self.regions), self)
        self.regions.append(region)
        rndVal = numpy.random.randint(0, 256, 3)
        self.region_colors.append((rndVal[0], rndVal[1], rndVal[2], 100))    
        
    def isInObsBkLinePair(self, pos):
        
        pPos = symgeo.Point(pos[0], pos[1])
        for bkline in self.obsBkPairLines:
            if bkline.contains(pPos):
                return True
        return False    
        
    def initVisualize(self):
        pygame.init()
        self.screen = pygame.display.set_mode((self.width,self.height))
        pygame.display.set_caption(self.mapfile)
        self.font = pygame.font.SysFont(None, 15)
        
        self.region_idx = 0
        self.subregion_idx = 0
        self.subsegment_idx = -1
        
    def updateVisualize(self):
        
        self.screen.fill((255,255,255))
        
        if len(self.regions) > 0:
            region = self.regions[self.region_idx]
            region_color = self.region_colors[self.region_idx]
            '''
            xs, ys = region.polygon.exterior.coords.xy
            for i in range(len(xs)-1):
                pygame.draw.line(self.screen, region_color, (xs[i], ys[i]), (xs[i+1], ys[i+1]), 6)
            '''
                
            if len(region.subregions) > 0:
                sub_xs, sub_ys = region.subregions[self.subregion_idx].polygon.exterior.coords.xy
                for i in range(len(sub_xs)-1):
                    pygame.draw.line(self.screen, region_color, (sub_xs[i], sub_ys[i]), (sub_xs[i+1], sub_ys[i+1]), 6)
                '''    
                for n_info in region.subregions[self.subregion_idx].neighbor_info:
                    pygame.draw.line(self.screen, SUBREGION_BORDER_COLOR, n_info[1].line_seg.coords[0], n_info[1].line_seg.coords[1], 10)  
                ''' 
                
            self.screen.blit(self.font.render(region.name+"-"+str(self.subregion_idx), True, region_color), (self.width-50, 15))
        
        for obs in self.obstacles:
            xs, ys = obs.polygon.exterior.coords.xy
            for i in range(len(xs)-1):
                pygame.draw.line(self.screen, OBSTACLE_COLOR, (xs[i], ys[i]), (xs[i+1], ys[i+1]), 2)

            pygame.draw.line(self.screen, ALPHA_COLOR, obs.alpha_seg.line_seg.coords[0], obs.alpha_seg.line_seg.coords[1], 2)
            pygame.draw.line(self.screen, BETA_COLOR, obs.beta_seg.line_seg.coords[0], obs.beta_seg.line_seg.coords[1], 2)
            
            for ac in obs.alpha_self_intsecs:
                pygame.draw.circle(self.screen, ALPHA_SELF_COLOR, ac, 3)
            for bc in obs.beta_self_intsecs:
                pygame.draw.circle(self.screen, BETA_SELF_COLOR, bc, 3)
            for ac in obs.alpha_obs_intsecs:
                pygame.draw.circle(self.screen, ALPHA_OTHER_COLOR, ac, 3)
            for bc in obs.beta_obs_intsecs:
                pygame.draw.circle(self.screen, BETA_OTHER_COLOR, bc, 3)
            pygame.draw.circle(self.screen, OBS_BK_COLOR, obs.bk, 3)
            self.screen.blit(self.font.render(obs.name, True, OBSTACLE_COLOR), obs.centroid)
            
        if len(self.subsegments) > 0 and self.subsegment_idx >= 0:
            subsegment = self.subsegments[self.subsegment_idx]
            pygame.draw.line(self.screen, SUBSEGMENT_COLOR, subsegment.line_seg.coords[0], subsegment.line_seg.coords[1], 10)
            self.screen.blit(self.font.render(subsegment.name, True, (0,0,0)), (15,15))
            
            
        pygame.draw.circle(self.screen, CENTER_POINT_COLOR, self.centralPoint, 3)
  
        for event in pygame.event.get():                
            if event.type == QUIT:
                pygame.quit()
                sys.exit()
            elif event.type == KEYDOWN:
                if event.key == pygame.K_UP:
                    self.region_idx += 1
                    self.subregion_idx = 0
                elif event.key == pygame.K_DOWN:
                    self.region_idx -= 1
                    self.subregion_idx = 0
                elif event.key == pygame.K_LEFT:
                    self.subregion_idx -= 1
                elif event.key == pygame.K_RIGHT:
                    self.subregion_idx += 1
                elif event.key == pygame.K_SPACE:
                    self.subsegment_idx += 1
                    
        if self.region_idx < 0:
            self.region_idx = len(self.regions)-1
        elif self.region_idx >= len(self.regions):
            self.region_idx = 0
        
        if self.subregion_idx < 0:
            self.subregion_idx = len(self.regions[self.region_idx].subregions) - 1
        elif self.subregion_idx >= len(self.regions[self.region_idx].subregions):
            self.subregion_idx = 0
            
        if self.subsegment_idx >= len(self.subsegments):
            self.subsegment_idx = -1
            
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

