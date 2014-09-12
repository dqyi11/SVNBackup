'''
Created on Sep 6, 2014

@author: daqing_yi
'''

from PIL import Image
from binarization import *
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from utilities import *


if __name__ == '__main__':
       
    img_filename = '0397.pgm'
    #img_filename = '020206_131612_bp001_folio_094_k639_1837.ppm'
    #img_filename = 'Declaration_Pg1of1_AC_crop.pgm'
    #img_filename = 'Scan_half_crop_norm_009_small.pgm'
    #img_filename = 'seq-4_small.pgm'
    
    img = Image.open(img_filename)
    
    img_width = img.size[0]
    img_height = img.size[1]
    print "W:"+str(img_width) + " H:" + str(img_height)
    
    img_data = np.array(img)
    
    #print img_data.shape

    hist, bin_edges = np.histogram(img_data, np.arange(256))
    
    #print hist
    threshold, intraclass_variances = otsu(hist, img_width*img_height)
    
    print "Threshold: " + str(threshold)    
    
    binary_data = binarize(img_data, threshold, 255)
    
    csv_filename = img_filename + ".csv"
    writeToCsv(csv_filename, binary_data)
    
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.imshow(img, cmap = cm.Greys_r)
    
    fig1 = plt.figure()
    ax1 = fig1.add_subplot(111)
    ax1.step(np.arange(len(intraclass_variances)), intraclass_variances)
    ax1.plot(threshold, intraclass_variances[threshold], 'rs')
    ax1.annotate(str(threshold), xy=(threshold, intraclass_variances[threshold]), xytext=(threshold+2, intraclass_variances[threshold]))
    
    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111)
    ax2.imshow(binary_data, cmap = cm.Greys_r)
    plt.show()
    
    
    