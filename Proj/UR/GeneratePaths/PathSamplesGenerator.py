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
from xml.dom import minidom

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
                
            configFile = "config-" + name + "-" + str(iterNum) + '.xml'
            objectiveFile = "obj-" + name + "-" + str(iterNum) + '.png'
            objectiveColorFile = objectiveFile + "-c.png"
            objectiveVizFile = "objViz-" + name + "-" + str(iterNum) + '.png'
            pathoutFile = 'pathout-' + name + "-" + str(iterNum) + '.txt'
            pathPicFile = pathoutFile + '.jpg'
            pathPicCostFile = pathPicFile + '.jpg'

            self.world.selectGoal()
            valDist = gmmCostMap(param, self.world)    
            vizCostMap(valDist, COST_DIR + "/" + objectiveFile, COSTVIZ_DIR + "/" + objectiveVizFile, COST_DIR + "/" + objectiveColorFile, False)
            
            self.genConfig(CONFIG_DIR + "/" + configFile, COST_DIR + "/" + objectiveFile, PATH_TXT_DIR + "/" + pathoutFile)
            
            command_str = "./rrtstar-viz-demo "+ CONFIG_DIR + "/" + configFile
            print command_str
            os.system(command_str)
            
            self.drawPath(PATH_TXT_DIR + "/" + pathoutFile, PATH_MAP_DIR + "/" + pathPicFile, "")
            self.drawPath(PATH_TXT_DIR + "/" + pathoutFile, PATH_MAP_DIR + "/" + pathPicCostFile, COST_DIR + "/" + objectiveColorFile)
            
            paramGnr.dumpXML(params[iterNum], PATH_MAP_DIR + "/" + pathPicFile + ".xml")
                
                
    def genConfig(self, configFile, objFile, pathOutputFile):
            
        if configFile == "":
            return        
        xmldoc = minidom.Document()
        root = xmldoc.createElement("root")
        world_obj = xmldoc.createElement("world")
        world_obj.setAttribute( "goal_x", str(self.world.goal[0]) )
        world_obj.setAttribute( "goal_y", str(self.world.goal[1]) )
        world_obj.setAttribute( 'map_filename', self.mapFile )
        world_obj.setAttribute( 'map_fullpath', './'+self.mapFile )
        world_obj.setAttribute( 'map_height', str(self.world.height) )
        world_obj.setAttribute( 'map_width', str(self.world.width) ) 
        world_obj.setAttribute( 'max_iteration_num', str( self.maxRun ) )
        world_obj.setAttribute( 'min_dist_enabled', str( int( False ) ) )
        world_obj.setAttribute( 'objective_file', objFile )
        world_obj.setAttribute( 'path_output_file', pathOutputFile )
        world_obj.setAttribute( 'segment_length',  str(self.segmentLength) )
        world_obj.setAttribute( 'start_x', str(self.world.init[0]) )
        world_obj.setAttribute( 'start_y', str(self.world.init[1]) )
        xmldoc.appendChild(root)
        root.appendChild(world_obj)
        xmldoc.writexml( open(configFile, 'w'), indent="  ", addindent="  ", newl="\n" )
        xmldoc.unlink()
        
    def drawPath(self, pathFile, drawPathFile, background_file=""):
        p = Path()
        if True == p.loadFromFile(pathFile):            
            if "" == background_file:
                self.worldViz.drawPath(p, drawPathFile)
            else:
                self.worldViz.drawPath(p, drawPathFile, background_file)
            
        

        
    
        
        
        
        