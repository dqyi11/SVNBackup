'''
Created on Apr 24, 2014

@author: walter
'''

from WorldFileReader import *
from FileUtils import *

import sys 

if __name__ == '__main__':
    
    #file = 'world_wall2.world'
    file = 'two_houses.world'
    app = QtGui.QApplication(sys.argv)
    reader = WorldFileReader(file,5)
    #reader.dump('two_houses.png')
    reader.initWorldInfo()
    reader.dumpWorldInfo('two_house_world-temp.xml',5)
    replaceFileHead("two_house_world-temp.xml", "two_house_world.xml")
    #reader.dumpWorldObstacle('world_wall2.dat')
    #reader.dumpWorldObstacle('two_houses.dat')
    sys.exit(app.exec_())
