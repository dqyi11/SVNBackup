'''
Created on Sep 10, 2014

@author: daqing_yi
'''

import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from utilities import *

if __name__ == '__main__':
    
    img_filename = 'Morphology_1.png'
    img = cv2.imread(img_filename, 0)
    kernel = np.ones((5,5), np.uint8)
    
    dilate_data = cv2.dilate(img, kernel, iterations=1)
    '''
    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111)
    ax2.set_title('Dilate')
    ax2.imshow(dilate_data, cmap = cm.Greys_r)
    '''
    cv2.imshow('Dilate', dilate_data)
    cv2.waitKey(0)
    writeToCsv(img_filename+'.dilate.REF.csv', dilate_data)
    
    erode_data = cv2.erode(img, kernel, iterations=1)
    '''
    fig3 = plt.figure()
    ax3 = fig3.add_subplot(111)
    ax3.set_title('Erode')
    ax3.imshow(erode_data, cmap = cm.Greys_r)
    '''
    cv2.imshow('Erode', erode_data)
    cv2.waitKey(0)
    writeToCsv(img_filename+'.erode.REF.csv', erode_data)
    
    open_data = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)
    '''
    fig4 = plt.figure()
    ax4 = fig4.add_subplot(111)
    ax4.set_title('Open')
    ax4.imshow(open_data, cmap = cm.Greys_r)
    '''
    cv2.imshow('Open', open_data)
    cv2.waitKey(0)
    writeToCsv(img_filename+'.open.REF.csv', open_data)
    
    close_data = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)
    '''
    fig5 = plt.figure()
    ax5 = fig5.add_subplot(111)
    ax5.set_title('Close')
    ax5.imshow(close_data, cmap = cm.Greys_r)
    
    plt.show()
    '''
    cv2.imshow('Close', close_data)
    cv2.waitKey(0)
    writeToCsv(img_filename+'.close.REF.csv', close_data)
    
    