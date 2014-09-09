'''
Created on Sep 8, 2014

@author: daqing_yi
'''
import numpy as np

def countNum(img_data, maskSize):
    data_width = img_data.shape[0]
    data_height = img_data.shape[1]
    maskNum = maskSize[0] * maskSize[1]
    
    countData = np.zeros(img_data.shape, np.int)
    maskSizeData = np.zeros(img_data.shape, np.int)
    
    half_width_range_size = (maskSize[0] - 1)/2
    half_height_range_size = (maskSize[1] - 1)/2
    
    for i in range(data_width):
        for j in range(data_height):
            width_range = [0, 0]
            height_range = [0, 0]
            if i >= half_width_range_size:
                width_range[0] = i - half_width_range_size
            else:
                width_range[0] = 0
            if i < data_width - half_width_range_size:
                width_range[1] = i + half_width_range_size
            else:
                width_range[1] = data_width - 1
            if j >= half_height_range_size:
                height_range[0] = j - half_height_range_size
            else:
                height_range[0] = 0
            if j < data_height - half_height_range_size:
                height_range[1] = j + half_height_range_size
            else:
                height_range[1] = data_height - 1    
                
            maskSizeData[i,j] = (width_range[1] - width_range[0] + 1)*(height_range[1] - height_range[0] + 1)
            for mi in range(width_range[0], width_range[1]+1):
                for mj in range(height_range[0], height_range[1]+1):
                    countData[i,j] += img_data[mi,mj]
                    
    return countData, maskSizeData

def dataThreshold(data, threshold, ratio=1.0):
    
    threshold_data = np.zeros(data.shape, int)
    data_width = data.shape[0]
    data_height = data.shape[1]
    
    for i in range(data_width):
        for j in range(data_height):
            #print str(data[i,j]) + " : " + str(threshold[i,j])
            if data[i,j] >= threshold[i,j]:
                threshold_data[i,j] = 1
            else:
                threshold_data[i,j] = 0
                
    return threshold_data
    
class NorphologicalFiltering(object):
    
    def __init__(self, img_data, maskSize):
        self.img_data = img_data
        self.maskSize = maskSize
        self.countData, self.maskSizeData = countNum(img_data, maskSize)    
    
    def dilate(self):
        return dataThreshold(self.countData, np.ones(self.countData.shape, int))
    
    def erode(self):
        return dataThreshold(self.countData, self.maskSizeData)
    
    def major(self):
        return dataThreshold(self.countData, self.maskSizeData, 0.5)
    
    def open(self):
        erosion = self.erode()
        return dataThreshold(erosion, np.ones(erosion.shape, int))
    
    def close(self):
        dilation = self.dilate()
        return dataThreshold(dilation, self.maskSizeData)
    
    
            
                
                
                
            
                
            
    
