'''
Created on Apr 24, 2014

@author: walter
'''

from WorldFileReader import *
import sys 

if __name__ == '__main__':
    
    #file = 'world_wall2.world'
    file = 'two_houses.world'
    app = QtGui.QApplication(sys.argv)
    reader = WorldFileReader(file,5)
    #reader.dump('two_houses.png')
    reader.initWorldInfo()
    #reader.dumpWorldInfo('world_wall2_world.xml')    
    #reader.dumpWorldObstacle('world_wall2.dat')
    reader.dumpWorldObstacle('two_houses.dat')
    sys.exit(app.exec_())
