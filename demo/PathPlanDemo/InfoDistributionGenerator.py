from ArrayDataVisualizer import *
import csv
import sys, string, os, time
import numpy as np

class InfoDistributionGenerator(object):

    def __init__(self, labelMgr, N):
        self.labelMgr = labelMgr
        #self.visualizer = ArrayDataVisualizer()
        self.diffV = None
        self.diffH = None
        
        self.sourceFilename = None
        self.outputFilename = None
        self.outputFilenameH = None
        self.outputFilenameV = None
        
        self.N = N
        self.dt = 0.1
        self.visc = 0.0
        
        self.source_width = 9
        self.source_angle = 0
        self.source_strength = 0.2
        self.source_slope = 45.0/np.pi
        
    def generateDistribution(self, obstacleFile):        
        self.sourceFilename = self.generateFilename() + ".dat"
        self.generateSourceFile(self.sourceFilename)
        self.outputFilename = self.generateFilename()
        self.runCFD(obstacleFile, self.sourceFilename, self.outputFilename)
        
    def runCFD(self, obstacleFilename, sourceFilename, outputName):
        
        cmd = "CFD "+str(self.N) + " " + str(self.dt) + " " + str(self.visc) + " " + obstacleFilename + " " + sourceFilename + " " + outputName + " 1" 
        print cmd
        os.system(cmd)
        
        time.sleep(5)
        
        self.readDataDiffusion(outputName)
    

    def generateSourceFile(self, filename):
        completed = False
        
        with open(filename, 'w') as sourceFile:
            sourceFile.writeline(str(len(self.labelMgr.features)))
            for feature in self.labelMgr.features:
                dataStr = "0, " + str(feature.pos[0]) + ", " + str(feature.pos[1]) + ", " + str(self.source_width) + ", "
                dataStr += str(self.source_angle) + ", " + str(self.source_strength) + ", " + str(self.source_slope)
                sourceFile.writeline(dataStr)
        
        return completed

    def readDataDiffusion(self, filename):                 
        filenameH = filename + "_HorVel.csv"
        filenameV = filename + "_VertVel.csv"
        
        self.diffH = []
        with open(filenameH) as csvfileH:
            xH = csv.reader(csvfileH, delimiter=' ', quotechar='|')
            for row in xH:
                #print row
                floatRow = [float(x) for x in row[0].split(',')]
                self.diffH.append(floatRow)
            print self.diffH
           
        self.diffV = []
        with open(filenameV) as cvsfileV:
            xV = csv.reader(cvsfileV, delimiter=' ', quotechar='|')
            for row in xV:
                floatRow = [float(x) for x in row[0].split(',')]
                self.diffV.append(floatRow)
            print self.diffV
            
        
    def generateFilename(self):
        name = str(int(time.time()))
        return name        
        
    def cleanFiles(self):
        pass
        
    

        