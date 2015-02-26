'''
Created on Feb 1, 2015

@author: daqing_yi
'''

import pygame, sys
from pygame.locals import *
import numpy
from TrajectoryReader import *

OBSTACLE_COLOR = (122,122,122)
ALPHA_COLOR = (0,0,255)
BETA_COLOR = (0,255,0)
ALPHA_SELF_COLOR = (153,204,255)
ALPHA_OTHER_COLOR = (0, 76, 153)
BETA_SELF_COLOR = (153,255,204)
BETA_OTHER_COLOR = (76,153,0)
CENTER_POINT_COLOR = (255,0,0)
OBS_BK_COLOR = (124,252,0)
HUMAN_PATH_COLOR = (255, 128, 0)
CONVERTED_HUMAN_PATH_COLOR = (102,51,0)

SEGMENT_LINE_WIDTH = 1
OBSTACLE_LINE_WIDTH = 2
KEYPOINT_SIZE = 2
HUMAN_PATH_WIDTH = 4


class InteractiveWorldMapVisualizer(object):


    def __init__(self, world_map):
        self.world_map = world_map
        self.tracking = False
        self.reader = TrajectoryReader(self.world_map)
        self.trackingPosList = []
        self.convertedTrackingPosList = []
        self.refString = []
        
        self.start = None
        self.end = None
        
    def initVisualize(self):
        pygame.init()
        self.screen = pygame.display.set_mode((self.world_map.width,self.world_map.height))
        pygame.display.set_caption(self.world_map.mapfile)
        self.font = pygame.font.SysFont(None, 15)
        
    def displayPolygon(self, polygon, polygon_color, line_width):
        xs, ys = polygon.exterior.coords.xy
        for i in range(len(xs)-1):
            pygame.draw.line(self.screen, polygon_color, (xs[i], ys[i]), (xs[i+1], ys[i+1]), line_width)
        
    def updateVisualize(self):
        
        for event in pygame.event.get():                
            if event.type == QUIT:
                pygame.quit()
                sys.exit()
            elif event.type == KEYDOWN:
                if event.key == pygame.K_x:
                    #self.convertedTrackingPosList = self.convertingPosList(self.trackingPosList)
                    self.convertedTrackingPosList, self.referenceInfo = self.reader.readPath(self.trackingPosList)
                    self.refString = self.reader.getString(self.referenceInfo)
                    
                    self.start =  self.referenceInfo[0] #self.world_map.findRegionByPoint( self.convertedTrackingPosList[0] )
                    self.end = self.referenceInfo[1] #self.world_map.findRegionByPoint( self.convertedTrackingPosList[len(self.convertedTrackingPosList)-1] )
            elif event.type == pygame.MOUSEBUTTONDOWN:
                self.trackingPosList = []
                self.convertedTrackingPosList = []
                self.tracking = True
            elif event.type == pygame.MOUSEBUTTONUP:
                self.tracking = False
            elif event.type == pygame.MOUSEMOTION:
                if self.tracking == True:
                    self.trackingPosList.append((event.pos[0], event.pos[1]))
                    #print "mouse at (%d, %d)" % event.pos
        
        self.screen.fill((255,255,255))
        
        for obs in self.world_map.obstacles:
            self.displayPolygon(obs.polygon, OBSTACLE_COLOR, OBSTACLE_LINE_WIDTH)

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
            
        for i in range(len(self.trackingPosList)-1):
            pygame.draw.line(self.screen, HUMAN_PATH_COLOR, self.trackingPosList[i], self.trackingPosList[i+1], HUMAN_PATH_WIDTH)
            
        for i in range(len(self.convertedTrackingPosList)-1):
            pygame.draw.line(self.screen, CONVERTED_HUMAN_PATH_COLOR, self.convertedTrackingPosList[i], self.convertedTrackingPosList[i+1], HUMAN_PATH_WIDTH)
            
        trackingPosListLen = len(self.trackingPosList) 
        if trackingPosListLen > 0:
            pygame.draw.circle(self.screen, (76,0,153), self.trackingPosList[0], 5)
            pygame.draw.rect(self.screen, (204,153,255), (self.trackingPosList[trackingPosListLen-1][0], self.trackingPosList[trackingPosListLen-1][1], 10, 10))
        
        if self.world_map.centralPoint != None:
            pygame.draw.circle(self.screen, CENTER_POINT_COLOR, self.world_map.centralPoint, KEYPOINT_SIZE)
  
        
        pygame.display.update()
    
    '''
    def convertingPosList(self, posList):
        convertedPosList = []
        if len(posList) > 0:
            
            currPos = posList[0]
            for i in range(1, len(posList)):
                if self.world_map.isCrossingObstalce(currPos, posList[i]):
                    convertedPosList.append(currPos)
                    currPos = posList[i]
        convertedPosList.append(posList[len(posList)-1])
                    
        return convertedPosList
    '''