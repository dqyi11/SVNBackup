'''
Created on Apr 24, 2014

@author: walter
'''

from WorldFileReader import *
import sys 

if __name__ == '__main__':
    
    file = 'two_house_with_enemy.world'
    #file = 'two_houses.world'
    app = QtGui.QApplication(sys.argv)
    reader = WorldFileReader(file,5)
    #reader.dump('two_houses.png')
    reader.initWorldInfo()
    reader.dumpWorldInfo('two_house_with_enemy_world.xml')    
    sys.exit(app.exec_())
