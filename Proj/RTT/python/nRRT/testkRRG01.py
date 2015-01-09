'''
Created on Jan 3, 2015

@author: daqing_yi
'''

from kRRG import *
from RRTVisualizer import *

if __name__ == '__main__':
    
    def calcDist(currentNode, referenceNode):
        dist = 0.0
        if referenceNode==None:
            return dist
        dist = np.sqrt((currentNode.pos[0]-referenceNode.pos[0])**2+(currentNode.pos[1]-referenceNode.pos[1])**2)
        return dist    
    
    MAP_FILE = './lab-map-inferior.png'
    MAP_FILE = './lab-map-scaled.png'
    krrg = kRRG([444, 989], 10)
    krrg.loadMap(MAP_FILE)
    
    krrg_viz = RRTVisualizer(krrg)
    
    krrg.init([40,40], [320, 300], calcDist)
    #rrt.init([40,40], [200, 200])
    
    for i in range(2000):
        krrg.extend()
        krrg_viz.update()
    