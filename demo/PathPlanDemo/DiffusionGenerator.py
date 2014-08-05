import csv
import sys, string, os

class DiffusionGenerator(object):

    def __init__(self, labelMgr):
        self.labelMgr = labelMgr
        self.diffV = None
        self.diffP = None
        
    def generateDataDiffusion(self):
        os.system("")
        
        filenameA = ""
        
        with open(filenameA) as csvfile:
            x = csv.reader(csvfile, delimiter=' ', quotechar='|')
        
        
        
        
        
