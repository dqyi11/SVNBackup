'''
Created on Sep 8, 2014

@author: daqing_yi
'''

from PIL import Image
from binarization import *
from morphology import *
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm

from binarization import *

if __name__ == '__main__':
    
    img = Image.open('0397.pgm')
    #img = Image.open('020206_131612_bp001_folio_094_k639_1837.ppm')
    #img = Image.open('Declaration_Pg1of1_AC_crop.pgm')
    #img = Image.open('Scan_half_crop_norm_009_small.pgm')
    #img = Image.open('seq-4_small.pgm')
    img_width = img.size[0]
    img_height = img.size[1]
    #print "W:"+str(img_width) + " H:" + str(img_height)
    
    img_data = np.array(img)
    
    #print img_data.shape

    hist, bin_edges = np.histogram(img_data, 256)
    
    #print hist
    threshold = otsu(hist, img_width*img_height)
    
    print "Threshold: " + str(threshold)    
    
    binary_data = binarize(img_data, threshold)
    
    mf = NorphologicalFiltering(binary_data, [3,3])
    open_data = mf.open()
    close_data = mf.close()
    
    
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.imshow(img, cmap = cm.Greys_r)
    
    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111)
    ax2.set_title('opening')
    ax2.imshow(open_data, cmap = cm.Greys_r)
    
    fig3 = plt.figure()
    ax3 = fig3.add_subplot(111)
    ax3.set_title('closing')
    ax3.imshow(close_data, cmap = cm.Greys_r)
    
    plt.show()