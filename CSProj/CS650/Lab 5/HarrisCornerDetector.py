'''
Created on 2014-11-19

@author: Walter
'''
import numpy as np
import cv2
import scipy.ndimage as ndimage
import scipy.ndimage.filters as filters

class HarrisCornerDetector(object):

    def __init__(self, size, threshold, min_distance):
        self.size = size
        self.threshold = threshold
        self.min_distance = min_distance
    
    def getCornerPoints(self, img):
        points = []
        
        img_dx = cv2.Sobel(img, cv2.CV_64F, 1, 0, ksize=3)
        img_dy = cv2.Sobel(img, cv2.CV_64F, 0, 1, ksize=3)
        
        Wxx = cv2.GaussianBlur(img_dx*img_dx, (self.size, self.size), 0)
        Wxy = cv2.GaussianBlur(img_dx*img_dy, (self.size, self.size), 0)
        Wyy = cv2.GaussianBlur(img_dy*img_dy, (self.size, self.size), 0)
        
        Wdet = Wxx*Wyy - Wxy*Wxy
        Wtr = Wxx + Wyy
        M = Wdet / (Wtr + 1e-6)
        
        corner_threshold = np.max(M.ravel()) * self.threshold
        peaks = self.findLocalMax(M, corner_threshold, self.min_distance)
        points = peaks
        return points
    
    def findLocalMax(self, data, threshold, min_distance):
    
        detected_peaks = []
        data_max = filters.maximum_filter(data, min_distance)
        maxima = (data == data_max)
        data_min = filters.minimum_filter(data, min_distance)
        diff = ((data_max - data_min) > threshold)
        maxima[diff == 0] = 0
        
        labeled, num_objects = ndimage.label(maxima)
        slices = ndimage.find_objects(labeled)
        x, y = [], []
        for dy,dx in slices:
            x_center = (dx.start + dx.stop - 1)/2
            y_center = (dy.start + dy.stop - 1)/2    
            detected_peaks.append([x_center, y_center])
    
        return detected_peaks
        
        
        
        
        