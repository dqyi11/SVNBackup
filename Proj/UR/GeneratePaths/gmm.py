'''
Created on Jul 30, 2015

@author: daqing_yi
'''
import numpy as np
from matplotlib import mpl,pyplot,cm
import matplotlib

def vizCostMap(data, outputfilename="", outputVizFilename="", blockVal=True):
    
    fig = pyplot.figure()
    ax = fig.add_subplot(111)
    img = ax.imshow(data)
    pyplot.colorbar(img)
    if outputVizFilename!="":
        pyplot.savefig(filename=outputVizFilename)
    if outputfilename!= "":
        matplotlib.image.imsave(outputfilename, np.uint8(data), cmap=cm.gray)
    #pyplot.show(block=blockVal)

class Param:
    
    def __init__(self):
        self.w = []
        self.scale = []

def gmmCostFunc(pos, param, world):
    
    val = 0.0
    obj_num = len(world.objects)
    for i in range(obj_num):
        obj = world.objects[i]
        x = pos[0] - obj.center.x
        y = pos[1] - obj.center.y
        dist = x**2 / np.abs(obj.bounding[2]-obj.bounding[0]) + y**2 / np.abs(obj.bounding[3]-obj.bounding[1])
        dist /= param.scale[i]
        val += param.w[i] * np.exp( - dist )
    
    return val

def gmmCostMap(param, world, type="Gaussian"):
    
    width = world.width
    height = world.height
    
    xs = np.arange(0,width,1)
    ys = np.arange(0,height,1)
    xv, yv = np.meshgrid(xs, ys)
    
    obj_num = len(world.objects)
    
    val = np.zeros(xv.shape)
    for i in range(obj_num):
        obj = world.objects[i]
        obs_x = obj.center.x
        obs_y = obj.center.y
        obs_w = np.abs(obj.bounding[2] - obj.bounding[0])
        obs_h = np.abs(obj.bounding[3] - obj.bounding[1])
        if type == "Gaussian":
            val += param.w[i] * np.exp( - ( (xv - obs_x)**2 / obs_w  + (yv - obs_y)**2 / obs_h ) / param.scale[i] )
        elif type == "Epanechnikov":
            dist = np.sqrt( (xv - obs_x)**2 / obs_w  + (yv - obs_y)**2 / obs_h ) 
            dz = (dist < param.scale[i])
            val += param.w[i] * ( 0.75 * (1 - dist) * dz.astype(float) )
        elif type == "Tricube":
            dist = np.sqrt( (xv - obs_x)**2 / obs_w  + (yv - obs_y)**2 / obs_h ) 
            dz = (dist < param.scale[i])
            val += param.w[i] * ( ( (70.0/81.0) * (1 - dist**(1.5))**3 ) * dz.astype(float) )
        elif type == "Logistic":
            dist = np.sqrt( (xv - obs_x)**2 / obs_w  + (yv - obs_y)**2 / obs_h ) / param.scale[i]
            v = np.exp( dist ) + 2 + np.exp( - dist )
            val += param.w[i] / v
        
    min_val = np.min(val)
    max_val = np.max(val)
    val = 255 * (val - min_val) / (max_val - min_val)    
        
    cols = np.unique(xv).shape[0]    
    val.reshape(-1, cols)
        
    return val.astype(int)
