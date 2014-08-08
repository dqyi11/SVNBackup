from InfoDistributionGenerator import *
from LabelManager import *
from ArrayDataVisualizer import *
import sys

if __name__ == '__main__':
    
    mgr = LabelManager()
    mgr.loadFile('two_house.xml')
    gnr = InfoDistributionGenerator(mgr, 500)
    gnr.generateDistribution('two_houses.dat')
    #gnr.runCFD('two_houses.dat', '1407529664.dat', 'testData')
    #gnr.readDataDiffusion('testData')
    
    vis = ArrayDataVisualizer(None)
    vis.setDataArray(500,500,gnr.diff)
    vis.dumpToFile('diff.png')
