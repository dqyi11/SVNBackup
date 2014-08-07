from InfoDistributionGenerator import *
from LabelManager import *
from ArrayDataVisualizer import *
import sys

if __name__ == '__main__':
    
    mgr = LabelManager()
    gnr = InfoDistributionGenerator(mgr, 500)
    #gnr.runCFD('two_houses.dat', 'two_house_source.dat', 'testData')
    gnr.readDataDiffusion('testData')  
      
    
    vis = ArrayDataVisualizer(None)
    vis.setDataArray(500,500,gnr.diffH)
    vis.dumpToFile('diffH.png')
