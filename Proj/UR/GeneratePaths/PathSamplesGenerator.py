'''
Created on Jul 31, 2015

@author: daqing_yi
'''

from World import *
from ParamGenerator import *
from gmm import *
from Path import *
import numpy as np
import json, os

CONFIG_PREFIX = "CONFIG"
COSTVIZ_PREFIX = "COSTVIZ"
PATH_COST_PREFIX = "PATH_COST"
PATH_MAP_PREFIX = "PATH_MAP"
PATH_TXT_PREFIX = "PATH_TXT"
COST_PREFIX = "COST"


class PathSamplesGenerator(object):

    def __init__(self, worldViz, mapFile, maxRunNum, segmentLen, folder="./"):
        
        self.worldViz = worldViz
        self.world = worldViz.world
        self.params = []
        self.jsonfiles = []
        self.objfiles = []
        self.objvizfiles = []
        self.mapFile = mapFile
        self.maxRun = maxRunNum
        self.segmentLength = segmentLen
        self.folder = folder
        
        
    def run(self, iterNumMax, name):
        
        CONFIG_DIR = self.folder + CONFIG_PREFIX
        COSTVIZ_DIR = self.folder + COSTVIZ_PREFIX
        PATH_COST_DIR = self.folder + PATH_COST_PREFIX
        PATH_MAP_DIR = self.folder + PATH_MAP_PREFIX
        PATH_TXT_DIR = self.folder + PATH_TXT_PREFIX
        COST_DIR = self.folder + COST_PREFIX
        if not os.path.exists(CONFIG_DIR):
            os.makedirs(CONFIG_DIR)
        if not os.path.exists(COSTVIZ_DIR):
            os.makedirs(COSTVIZ_DIR)
        if not os.path.exists(PATH_COST_DIR):
            os.makedirs(PATH_COST_DIR)
        if not os.path.exists(PATH_MAP_DIR):
            os.makedirs(PATH_MAP_DIR)
        if not os.path.exists(PATH_TXT_DIR):
            os.makedirs(PATH_TXT_DIR)
        if not os.path.exists(COST_DIR):
            os.makedirs(COST_DIR)
            
        paramGnr = ParamGenerator(self.worldViz)
        params = paramGnr.generateParams(iterNumMax)    
        
        for iterNum in range(iterNumMax):
            
            print "At iteration " + str(iterNum)
            
            param = params[iterNum]
                
            configFile = "config-" + name + "-" + str(iterNum) + '.json'
            objectiveFile = "obj-" + name + "-" + str(iterNum) + '.png'
            objectiveVizFile = "objViz-" + name + "-" + str(iterNum) + '.png'
            pathoutFile = 'pathout-' + name + "-" + str(iterNum) + '.txt'
            pathPicFile = pathoutFile + '.jpg'

            self.world.selectGoal()
            valDist = gmmCostMap(param, self.world)    
            vizCostMap(valDist, COST_DIR + "/" + objectiveFile, COSTVIZ_DIR + "/" + objectiveVizFile, False)
            self.genConfig(CONFIG_DIR + "/" + configFile, COST_DIR + "/" + objectiveFile, PATH_TXT_DIR + "/" + pathoutFile)
            
            command_str = "./rrtstar_viz_demo "+ CONFIG_DIR + "/" + configFile
            print command_str
            os.system(command_str)
            
            self.drawPath(PATH_TXT_DIR + "/" + pathoutFile, PATH_MAP_DIR + "/" + pathPicFile)
            
            
                
                
    def genConfig(self, configFile, objFile, pathOutputFile):
        my_dict = {
            'goalX'           : self.world.goal[0],
            'goalY'           : self.world.goal[1],
            'mapFilename'     : self.mapFile,
            'mapFullpath'     : './'+self.mapFile,
            'mapHeight'       : self.world.height,
            'mapWidth'        : self.world.width,
            'maxIterationNum' : self.maxRun,
            'minDistEnabled'  : False,
            'objectiveFile'   : objFile,
            'pathOutputFile'  : pathOutputFile,
            'segmentLength'   : self.segmentLength,
            'startX'          : self.world.init[0],
            'startY'          : self.world.init[1]
        }
        with open(configFile, 'w') as outfile:
            json.dump(my_dict, outfile)
        
        
    def drawPath(self, pathFile, drawPathFile):
        p = Path()
        if True == p.loadFromFile(pathFile):
            self.worldViz.drawPath(p, drawPathFile)
        

        
    
        
        
        
        