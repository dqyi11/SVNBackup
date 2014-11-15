'''
Created on Nov 3, 2014

@author: daqing_yi
'''

from PIL import Image
import numpy as np
from ConnectedComponent import *
import csv
import pygame, sys
from pygame.locals import *
from Shapes import *

class Obstacle(object):
    
    def __init__(self, idx, pixels):
        self.idx = idx
        self.pixels = pixels
        self.center = None
        self.keypoint = None
        self.alpha_line = None
        self.beta_line = None
        
        self.alpha_lines = []
        self.beta_lines = []
        
        self.min_x = np.inf
        self.max_x = -1
        self.min_y = np.inf
        self.max_y = -1
        
        for p in self.pixels:
            if p[0] < self.min_x:
                self.min_x = p[0]
            if p[0] > self.max_x:
                self.max_x = p[0]
            if p[1] < self.min_y:
                self.min_y = p[1]
            if p[1] > self.max_y:
                self.max_y = p[1]
        
        self.center = self.getCenter()
        self.keypoint = self.samplePosition()
        
        rad = np.arctan2(float(self.alpha_line[0][0]-self.apha_line[1][0]), float(self.alpha_line[0][0]-self.apha_line[1][0]))
        if rad < 0:
            rad += np.pi*2
        self.alpha_rad = rad
        
        rad = np.arctan2(float(self.beta_line[0][0]-self.beta_line[1][0]), float(self.beta_line[0][0]-self.beta_line[1][0]))
        if rad < 0:
            rad += np.pi*2
        self.beta_rad = rad
        
        
        
    def samplePosition(self):
        rndIdx = np.random.randint(len(self.pixels))
        return self.pixels[rndIdx]
    
    def getCenter(self):
        mX, mY = 0.0, 0.0
        for p in self.pixels:
            mX += p[0]
            mY += p[1]
        mX /= len(self.pixels)
        mY /= len(self.pixels)
        return [int(mX), int(mY)]
    
    def hasPoint(self, point):
        for p in self.pixels:
            if p[0]==point[0] and p[1]==point[1]:
                return True
        return False

    def intersectWithLine(self, line):
        for x in range(self.min_x, self.max_x+1):
            y = line.getY(x)
            if self.hasPoint([x,y]):
                return True
        return False
        
    def initLine(self, map_center, map_size):
        t_line = Line(map_center, self.keypoint)
        y_minx = t_line.getY(0)
        x_miny = t_line.getX(0)
        y_maxx = t_line.getY(map_size[0]-1)
        x_maxy = t_line.getX(map_size[1]-1)

        if map_center[0] <= self.keypoint[0]:
            if map_center[1] <= self.keypoint[1]:
                #mode = 4   keypoint in 4th               
                if y_minx >= 0:
                    self.alpha_line = [[self.keypoint[0],self.keypoint[1]], [0, y_minx]]
                elif x_miny >= 0:
                    self.alpha_line = [[self.keypoint[0],self.keypoint[1]], [x_miny, 0]]  
                
                if y_maxx < map_size[1]:
                    self.beta_line = [[self.keypoint[0],self.keypoint[1]], [map_size[0]-1, y_maxx]]
                elif x_maxy < map_size[0]:
                    self.beta_line = [[self.keypoint[0],self.keypoint[1]], [x_maxy, map_size[1]-1]]
  
            elif map_center[1] > self.keypoint[1]:
                #mode = 1
                if y_minx < map_size[1]:
                    self.alpha_line = [[self.keypoint[0],self.keypoint[1]], [0, y_minx]]
                elif x_maxy >= 0:
                    self.alpha_line = [[self.keypoint[0],self.keypoint[1]], [x_maxy, map_size[1]]]
                
                if y_maxx >= 0:
                    self.beta_line = [[self.keypoint[0],self.keypoint[1]], [map_size[0]-1, y_maxx]]
                elif x_miny < map_size[0]:
                    self.beta_line = [[self.keypoint[0],self.keypoint[1]], [x_miny, 0]]   

        elif map_center[0] > self.keypoint[0]:
            if map_center[1] <= self.keypoint[1]:
                #mode = 3
                if y_maxx >= 0:
                    self.alpha_line = [[self.keypoint[0],self.keypoint[1]], [map_size[0]-1,y_maxx]]
                elif x_miny < map_size[0]:
                    self.alpha_line = [[self.keypoint[0],self.keypoint[1]], [x_miny, 0]]
                
                if y_minx < map_size[1]:
                    self.beta_line = [[self.keypoint[0],self.keypoint[1]], [0, y_minx]]
                elif x_maxy >= 0:
                    self.beta_line = [[self.keypoint[0],self.keypoint[1]], [x_maxy, map_size[1]-1]]   
            elif map_center[1] > self.keypoint[1]:
                #mode = 2
                if y_maxx < map_size[1]:
                    self.alpha_line = [[self.keypoint[0],self.keypoint[1]], [map_size[0]-1, y_maxx]]
                elif x_maxy < map_size[0]:
                    self.alpha_line = [[self.keypoint[0],self.keypoint[1]], [x_maxy, map_size[1]-1]]
                
                if y_minx >= 0:
                    self.beta_line = [[self.keypoint[0],self.keypoint[1]], [0, y_minx]]
                elif x_miny >= 0:
                    self.beta_line = [[self.keypoint[0],self.keypoint[1]], [x_miny, 0]]           
    
    

    
class ReferenceFrameManager(object):
    
    def __init__(self, center_pos, obs_pos):
        self.center_pos = center_pos
        self.obs_pos = obs_pos
        self.k = float(self.obs_pos[0]-self.center_pos[0])/(self.obs_pos[1]-self.center_pos[1])
        #print self.k
        self.quarant = 0
        if self.center_pos[0] < self.obs_pos[0]:
            if self.center_pos[1] >= self.obs_pos[1]:
                self.quarant = 4
        else:
            if self.center_pos[1] >= self.obs_pos[1]:
                self.quarant = 3
            else:
                self.quarant = 2
        

class WorldMap(object):

    def __init__(self):
        self.obstacle_num = 0
        self.obstacles = []

    def load(self, map_file):
        img = Image.open(map_file).convert("L")
        self.width = img.size[0]
        self.height = img.size[1]
        self.bin_data = np.zeros((self.width, self.height), np.int)
        for i in range(self.width):
            for j in range(self.height):
                if img.getpixel((i,j)) <= 122:
                    self.bin_data[i,j] = 0
                else:
                    self.bin_data[i,j] = 1
        
        ccMgr = ConnectedComponentMgr(self.bin_data)
        self.obstacle_num = ccMgr.getComponentNum()
        for idx in range(self.obstacle_num):
            obs = Obstacle(idx, ccMgr.getComponent(idx))
            self.obstacles.append(obs)
        
        mX, mY = 0.0, 0.0
        for obs in self.obstacles:
            mX += obs.center[0]
            mY += obs.center[1]
        mX /= len(self.obstacles)
        mY /= len(self.obstacles)
        self.obsCenter = [int(mX), int(mY)]
        
        for obs in self.obstacles:
            obs.initLine(self.obsCenter, [self.width, self.height])
            
        for obs in self.obstacles:
            obs.alpha_lines = self.segementLineByObstacles(Line(obs.alpha_line[0],obs.alpha_line[1]))
            obs.beta_lines = self.segementLineByObstacles(Line(obs.beta_line[0], obs.beta_line[1]))
        
        self.rf_mgrs = []
        for obs in self.obstacles:
            rf_mgr = ReferenceFrameManager(obs.keypoint, self.obsCenter)
            self.rf_mgrs.append(rf_mgr)
        
    def segementLineByObstacles(self, line):
        segment_point_list = []
        current_pixel_obstacle = True
        prev_pixel_obstacle = True
        start = [line.min_x, line.getY(line.min_x)]
        end = [line.max_x, line.getY(line.max_x)]
        for x in range(line.min_x, line.max_x+1):      
            y = line.getY(x)
            current_pixel_obstacle = False
            for obs in self.obstacles:
                if obs.hasPoint([x,y]):
                    current_pixel_obstacle = True
                    break
                
            if current_pixel_obstacle == True:
                if prev_pixel_obstacle == False:
                    segment_point_list.append([x,y])
        
            prev_pixel_obstacle = current_pixel_obstacle
            
        lines = []
        point_num = len(segment_point_list)
        print "point num " + str(point_num)
        
        if point_num == 0:
            lines.append([start, end])     
        else:
            lines.append([start, segment_point_list[0]])
            for idx in range(0, point_num-1, 1):
                lines.append([segment_point_list[idx], segment_point_list[idx+1]])
            lines.append([segment_point_list[point_num-1], end])
            
        return lines                

        
    def dump(self, filename):
        with open(filename, 'wb') as f:
            writer = csv.writer(f)
            writer.writerows(self.bin_data)
        
    def visualize(self):
        pygame.init()
        self.screen = pygame.display.set_mode((self.width,self.height))
        pygame.display.set_caption('Topological graph')
        self.screen.fill((255,255,255))
        
        #pygame.draw.circle(self.screen, (125,255,255),[10,10],10)
        
        for obs in self.obstacles:
            for o in obs.pixels:
                self.screen.set_at((o[0], o[1]), (122,122,122))
        for obs in self.obstacles:  
            '''  
            if obs.alpha_line != None:
                pygame.draw.line(self.screen, (0,255,0), obs.alpha_line[0], obs.alpha_line[1], 2)
            if obs.beta_line != None:
                pygame.draw.line(self.screen, (0,0,255), obs.beta_line[0], obs.beta_line[1], 2)
            '''
            for alpha in obs.alpha_lines:
                pygame.draw.line(self.screen, (0,255,0), alpha[0], alpha[1], 2)
                pygame.draw.circle(self.screen, (124,252,0), alpha[0], 8)
                pygame.draw.circle(self.screen, (25,25,112), alpha[1], 6)
            for beta in obs.beta_lines:
                pygame.draw.line(self.screen, (0,0,255), beta[0], beta[1], 2)
                pygame.draw.circle(self.screen, (34,139,34), beta[0], 8)
                pygame.draw.circle(self.screen, (135,206,250), beta[1], 6)
        for obs in self.obstacles:       
            pygame.draw.circle(self.screen, (255,0,0), [obs.keypoint[0],obs.keypoint[1]],3)

        pygame.draw.circle(self.screen, (0,0,0), [self.obsCenter[0], self.obsCenter[1]],3)
        
        while True:
            for event in pygame.event.get():
                if event.type == QUIT:
                    pygame.quit()
                    sys.exit()
            pygame.display.update()

        