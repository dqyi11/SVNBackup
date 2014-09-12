'''
Created on Sep 10, 2014

@author: daqing_yi
'''

import cv2
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.cm as cm
from binarization import *
from utilities import *

if __name__ == '__main__':
    
    img_filename = '0397.pgm'
    #img_filename = '020206_131612_bp001_folio_094_k639_1837.ppm'
    #img_filename = 'Declaration_Pg1of1_AC_crop.pgm'
    #img_filename = 'Scan_half_crop_norm_009_small.pgm'
    #img_filename = 'seq-4_small.pgm'
    
    img = cv2.imread(img_filename,0) 

    img_width = img.shape[0]
    img_height = img.shape[1]
    print "W:"+str(img_width) + " H:" + str(img_height)
    
    img_data = np.array(img)
    
    #print img_data.shape
    ret,threshold = cv2.threshold(img,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    
    csv_filename = img_filename + ".REF.csv"
    writeToCsv(csv_filename, threshold)
    
    print ret
    
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.imshow(img, cmap = cm.Greys_r)
    
    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111)
    ax2.imshow(threshold, cmap = cm.Greys_r)
    plt.show()