from LabelManager import *

if __name__ == '__main__':
    
    mgr = LabelManager()
    mgr.loadFile('two_house.xml')
    mgr.printLabels()