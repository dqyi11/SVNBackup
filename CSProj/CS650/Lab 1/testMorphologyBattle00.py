'''
Created on Sep 14, 2014

@author: daqing_yi
'''

import cv2
from PIL import Image
from binarization import *
from morphology import *
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from utilities import *

if __name__ == '__main__':
    
    img_filename = 'Morphology_1.png'

    img = Image.open(img_filename).convert("L")
    img_width = img.size[0]
    img_height = img.size[1]
    img_data = np.array(img)
    hist, bin_edges = np.histogram(img_data, np.arange(256))
    threshold, intraclass_variances = otsu(hist, img_width*img_height)
    binary_data = binarize(img_data, threshold)
    maskSize = [5,5]
    mf = NorphologicalFiltering(binary_data, maskSize)
    
    img_ref = cv2.imread(img_filename, 0)
    kernel = np.ones((5,5), np.uint8)
    
    dilate_data = mf.dilate()
    dilate_data_ref = cv2.erode(img_ref, kernel)
    dilate_data_cmp = dilate_data_ref - dilate_data
    
    erode_data = mf.erode()
    erode_data_ref = cv2.dilate(img_ref, kernel)
    erode_data_cmp = erode_data_ref - erode_data
    
    open_data = mf.open()
    open_data_ref = cv2.morphologyEx(img_ref, cv2.MORPH_CLOSE, kernel)
    open_data_cmp = open_data_ref - open_data
    
    close_data = mf.close()
    close_data_ref = cv2.morphologyEx(img_ref, cv2.MORPH_OPEN, kernel)
    close_data_cmp = close_data_ref - close_data
    
    print "DILATE: " + str(np.mean(np.abs(dilate_data_cmp)))
    print "ERODE: " + str(np.mean(np.abs(erode_data_cmp)))
    print "OPEN: " + str(np.mean(np.abs(open_data_cmp)))
    print "CLOSE: " + str(np.mean(np.abs(close_data_cmp)))