from VisibilityDataMgr import *

if __name__ == '__main__':
    
    mgr = VisibilityDataMgr()
    mgr.loadFile("visibleData.txt")
    mgr.dumpData('visData2.txt')