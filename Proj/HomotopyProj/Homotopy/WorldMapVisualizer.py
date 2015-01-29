'''
Created on Jan 29, 2015

@author: daqing_yi
'''

import pygame, sys
from pygame.locals import *
import numpy

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

SEGMENT_LINE_WIDTH = 1
OBSTACLE_LINE_WIDTH = 2
KEYPOINT_SIZE = 2
BORDER_LINE_WIDTH = 10
REGION_LINE_WIDTH = 6

class WorldMapVisualizer(object):


    def __init__(self, world_map):
        self.world_map = world_map
        
    def initVisualize(self):
        pygame.init()
        self.screen = pygame.display.set_mode((self.world_map.width,self.world_map.height))
        pygame.display.set_caption(self.world_map.mapfile)
        self.font = pygame.font.SysFont(None, 15)
        
        self.region_idx = 0
        self.subregion_idx = 0
        self.subsegment_idx = -1
        
        self.region_colors = []
        for i in range(self.world_map.regionNum):
            rndVal = numpy.random.randint(0, 256, 3)
            self.region_colors.append((rndVal[0], rndVal[1], rndVal[2]))
        
    def updateVisualize(self):
        
        self.screen.fill((255,255,255))
        
        if len(self.world_map.regions) > 0:
            region = self.world_map.regions[self.region_idx]
            region_color = self.region_colors[self.region_idx]
            '''
            xs, ys = region.polygon.exterior.coords.xy
            for i in range(len(xs)-1):
                pygame.draw.line(self.screen, region_color, (xs[i], ys[i]), (xs[i+1], ys[i+1]), 6)
            '''
                
            if len(region.subregions) > 0:
                sub_xs, sub_ys = region.subregions[self.subregion_idx].polygon.exterior.coords.xy
                for i in range(len(sub_xs)-1):
                    pygame.draw.line(self.screen, region_color, (sub_xs[i], sub_ys[i]), (sub_xs[i+1], sub_ys[i+1]), REGION_LINE_WIDTH)
                '''    
                for n_info in region.subregions[self.subregion_idx].neighbor_info:
                    pygame.draw.line(self.screen, SUBREGION_BORDER_COLOR, n_info[1].line_seg.coords[0], n_info[1].line_seg.coords[1], 10)  
                '''
                
            self.screen.blit(self.font.render(region.name+"-"+str(self.subregion_idx), True, region_color), (self.world_map.width-50, 15))
        
        for obs in self.world_map.obstacles:
            xs, ys = obs.polygon.exterior.coords.xy
            for i in range(len(xs)-1):
                pygame.draw.line(self.screen, OBSTACLE_COLOR, (xs[i], ys[i]), (xs[i+1], ys[i+1]), OBSTACLE_LINE_WIDTH)
            if obs.alpha_seg != None:
                pygame.draw.line(self.screen, ALPHA_COLOR, obs.alpha_seg.line_seg.coords[0], obs.alpha_seg.line_seg.coords[1], SEGMENT_LINE_WIDTH)
            if obs.beta_seg != None:
                pygame.draw.line(self.screen, BETA_COLOR, obs.beta_seg.line_seg.coords[0], obs.beta_seg.line_seg.coords[1], SEGMENT_LINE_WIDTH)
            
            for ac in obs.alpha_self_intsecs:
                pygame.draw.circle(self.screen, ALPHA_SELF_COLOR, ac, KEYPOINT_SIZE)
            for bc in obs.beta_self_intsecs:
                pygame.draw.circle(self.screen, BETA_SELF_COLOR, bc, KEYPOINT_SIZE)
            for ac in obs.alpha_obs_intsecs:
                pygame.draw.circle(self.screen, ALPHA_OTHER_COLOR, (int(ac[0]),int(ac[1])), KEYPOINT_SIZE)
            for bc in obs.beta_obs_intsecs:
                pygame.draw.circle(self.screen, BETA_OTHER_COLOR, (int(bc[0]),int(bc[1])), KEYPOINT_SIZE)
            if obs.bk != None:
                pygame.draw.circle(self.screen, OBS_BK_COLOR, obs.bk, KEYPOINT_SIZE)
            self.screen.blit(self.font.render(obs.name, True, OBSTACLE_COLOR), obs.centroid)
            
        if len(self.world_map.subsegments) > 0 and self.subsegment_idx >= 0:
            subsegment = self.world_map.subsegments[self.subsegment_idx]
            pygame.draw.line(self.screen, SUBSEGMENT_COLOR, subsegment.line_seg.coords[0], subsegment.line_seg.coords[1], BORDER_LINE_WIDTH)
            self.screen.blit(self.font.render(subsegment.name, True, (0,0,0)), (15,15))
            
        
        if self.world_map.centralPoint != None:
            pygame.draw.circle(self.screen, CENTER_POINT_COLOR, self.world_map.centralPoint, KEYPOINT_SIZE)
  
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
            self.region_idx = len(self.world_map.regions)-1
        elif self.region_idx >= len(self.world_map.regions):
            self.region_idx = 0
        
        if len(self.world_map.regions) > 0:
            if self.subregion_idx < 0:
                self.subregion_idx = len(self.world_map.regions[self.region_idx].subregions) - 1
            elif self.subregion_idx >= len(self.world_map.regions[self.region_idx].subregions):
                self.subregion_idx = 0
                
            if self.subsegment_idx >= len(self.world_map.subsegments):
                self.subsegment_idx = -1
            
        pygame.display.update()
