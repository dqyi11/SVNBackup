'''
Created on Sep 14, 2014

@author: daqing_yi
'''
import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from utilities import *

if __name__ == '__main__':
    
    #img_filename = '0397.pgm'
    img_filename = '020206_131612_bp001_folio_094_k639_1837.ppm'
    #img_filename = 'Declaration_Pg1of1_AC_crop.pgm'
    #img_filename = 'Scan_half_crop_norm_009_small.pgm'
    #img_filename = 'seq-4_small.pgm'
    img_data = cv2.imread(img_filename, 0)
    ret,img = cv2.threshold(img_data,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    kernel = np.ones((5,5), np.uint8)
    
    dilate_data = cv2.dilate(img, kernel, iterations=1)
    '''
    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111)
    ax2.set_title('Dilate')
    ax2.imshow(dilate_data, cmap = cm.Greys_r)
    '''
    #cv2.imshow('Dilate', dilate_data)
    cv2.imwrite(img_filename+'.ref.dilate.png', dilate_data)
    #cv2.waitKey(0)
    #writeToCsv(img_filename+'.dilate.REF.csv', dilate_data)
    
    erode_data = cv2.erode(img, kernel, iterations=1)
    '''
    fig3 = plt.figure()
    ax3 = fig3.add_subplot(111)
    ax3.set_title('Erode')
    ax3.imshow(erode_data, cmap = cm.Greys_r)
    '''
    #cv2.imshow('Erode', erode_data)
    cv2.imwrite(img_filename+'.ref.erode.png', erode_data)
    #cv2.waitKey(0)
    #writeToCsv(img_filename+'.erode.REF.csv', erode_data)
    
    open_data = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)
    '''
    fig4 = plt.figure()
    ax4 = fig4.add_subplot(111)
    ax4.set_title('Open')
    ax4.imshow(open_data, cmap = cm.Greys_r)
    '''
    #cv2.imshow('Open', open_data)
    cv2.imwrite(img_filename+'.ref.open.png', open_data)
    #cv2.waitKey(0)
    #writeToCsv(img_filename+'.open.REF.csv', open_data)
    
    close_data = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)
    '''
    fig5 = plt.figure()
    ax5 = fig5.add_subplot(111)
    ax5.set_title('Close')
    ax5.imshow(close_data, cmap = cm.Greys_r)
    
    plt.show()
    '''
    #cv2.imshow('Close', close_data)
    cv2.imwrite(img_filename+'.ref.close.png', close_data)
    #cv2.waitKey(0)
    #writeToCsv(img_filename+'.close.REF.csv', close_data)
    
    