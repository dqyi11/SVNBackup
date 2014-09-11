'''
Created on Sep 6, 2014

@author: daqing_yi
'''

from PIL import Image
from binarization import *
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm


if __name__ == '__main__':
    
    #img = Image.open('0397.pgm')
    #img = Image.open('020206_131612_bp001_folio_094_k639_1837.ppm')
    #img = Image.open('Declaration_Pg1of1_AC_crop.pgm')
    #img = Image.open('Scan_half_crop_norm_009_small.pgm')
    img = Image.open('seq-4_small.pgm')
    img_width = img.size[0]
    img_height = img.size[1]
    print "W:"+str(img_width) + " H:" + str(img_height)
    
    img_data = np.array(img)
    
    #print img_data.shape

    hist, bin_edges = np.histogram(img_data, 256)
    
    #print hist
    threshold = otsu(hist, img_width*img_height)
    
    print "Threshold: " + str(threshold)    
    
    binary_data = binarize(img_data, threshold)
    
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.imshow(img, cmap = cm.Greys_r)
    
    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111)
    ax2.imshow(binary_data, cmap = cm.Greys_r)
    plt.show()
    
    
    