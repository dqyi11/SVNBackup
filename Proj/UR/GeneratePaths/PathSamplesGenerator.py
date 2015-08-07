'''
Created on Jul 31, 2015

@author: daqing_yi
'''

from World import *
from gmm import *
import numpy as np
import json, os

class PathSamplesGenerator(object):

    def __init__(self, world, mapFile, maxRunNum, segmentLen, folder="./"):
        
        self.world = world
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
                param.scale.append(np.random.rand()*30+ 50)
                
            configFile = "config-" + str(iterNum) + '.json'
            objectiveFile = "obj-" + str(iterNum) + '.png'
            objectiveVizFile = "objViz-" + str(iterNum) + '.png'
            pathoutFile = 'pathout-' + str(iterNum) + '.txt'
            
            valDist = gmmCostMap(param, self.world)    
            vizCostMap(valDist, self.folder + objectiveFile, self.folder + objectiveVizFile, False)
            self.genConfig(self.folder + configFile, self.folder + objectiveFile, self.folder + pathoutFile)
            
            command_str = self.folder + "rrtstar_viz_demo "+ self.folder + configFile
            print command_str
            os.system(command_str)
                
                
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
        
        out_file = open(configFile, "w")
        json.dump(my_dict, out_file, indent=4)
        out_file.close()
                

        
    
        
        
        
        