'''
Created on Jul 31, 2015

@author: daqing_yi
'''

from World import *
from gmm import *
from Path import *
import numpy as np
import json, os

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
        
        
    def run(self, iterNumMax):
        
        for iterNum in range(iterNumMax):
            
            print "At iteration " + str(iterNum)
            
            param = Param()
            for j in range(len(self.world.objects)):
                param.w.append(np.random.rand()*2 - 1.0)
                param.scale.append(np.random.rand()*80+ 100)
                
            configFile = "config-" + str(iterNum) + '.json'
            objectiveFile = "obj-" + str(iterNum) + '.png'
            objectiveVizFile = "objViz-" + str(iterNum) + '.png'
            pathoutFile = 'pathout-' + str(iterNum) + '.txt'
            pathPicFile = pathoutFile + '.jpg'
            
            valDist = gmmCostMap(param, self.world)    
            vizCostMap(valDist, self.folder + objectiveFile, self.folder + objectiveVizFile, False)
            self.genConfig(self.folder + configFile, self.folder + objectiveFile, self.folder + pathoutFile)
            
            command_str = self.folder + "rrtstar_viz_demo "+ self.folder + configFile
            print command_str
            os.system(command_str)
            
            self.drawPath(pathoutFile, pathPicFile)
            
            
                
                
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
        p.loadFromFile(pathFile)
        self.worldViz.drawPath(p, drawPathFile)
        

        
    
        
        
        
        