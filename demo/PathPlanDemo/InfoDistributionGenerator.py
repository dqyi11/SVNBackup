from ArrayDataVisualizer import *
import csv
import sys, string, os, time

class InfoDistributionGenerator(object):

    def __init__(self, labelMgr):
        self.labelMgr = labelMgr
        self.visualizer = ArrayDataVisualizer()
        self.diffV = None
        self.diffH = None
        
        self.sourceFilename = None
        self.outputFilename = None
        self.outputFilenameH = None
        self.outputFilenameV = None
        
        self.N = 0
        self.dt = 0
        self.visc = 0.0
        
                
    
    def generateDistribution(self, obstacleFile):
        
        self.sourceFilename = self.generateFilename() + ".dat"
        self.generateSourceFile(self.sourceFilename)
        self.outputFilename = self.generateFilename()
        
        cmd = "cfd "+str(self.N) + " " + str(self.dt) + " " + str(self.visc) + " " + self.obstacleFile + " " + self.sourceFilename + " " + self.outputFilename
        print cmd
        os.system(cmd)
    

    def generateSourceFile(self, filename):
        
        completed = False
        
        with open(filename, 'w') as sourceFile:
            pass
        
        return completed

        
    def readDataDiffusion(self, filename):         
        
        
        filenameA = ""
        
        with open(filenameA) as csvfile:
            x = csv.reader(csvfile, delimiter=' ', quotechar='|')
            
    def generateFilename(self):
        name = str(int(time.time()))
        return name        
        
    def cleanFiles(self):
        
        pass
        
    

        