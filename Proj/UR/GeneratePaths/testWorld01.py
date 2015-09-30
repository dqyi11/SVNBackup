'''
Created on Jul 30, 2015

@author: daqing_yi
'''

from World import *
from WorldViz import *
from gmm import *


if __name__ == '__main__':
    
    world01 = World()
    world01.fromXML("world01.xml")
    '''
    worldViz01 = WorldViz(world01)
    cont = True
    while cont==True:
        cont = worldViz01.update()
    worldViz01.close()
    '''
    param = Param()
    for j in range(len(world01.objects)):
        param.w.append(np.random.rand()*2 - 1.0)
        param.scale.append(10.0)
        
    #KERNEL = "Epanechnikov"    
    #KERNEL = "Gaussian"
    #KERNEL = "Tricube"
    KERNEL = "Logistic"
    NAME = "world01"    
    valDist = gmmCostMap(param, world01, KERNEL)    
    vizCostMap(valDist, NAME+"-"+KERNEL+".png", NAME+"Viz-"+KERNEL+".png", False)
    
    '''
    valDist = np.zeros((world01.width, world01.height))
    xs = np.arange(0,world01.width,1)
    ys = np.arange(0,world01.height,1)
    xv, yv = np.meshgrid(xs, ys)
    for i in range(world01.width):
        for j in range(world01.height):
            valDist[i,j] = gmmCostFunc([i,j], param, world01)
            
    vizCostMap(valDist.T)
    '''