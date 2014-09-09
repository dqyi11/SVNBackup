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
    
    img = Image.open('Morphology_1.png').convert("L")

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
    
    mf = NorphologicalFiltering(binary_data, [5,5])
    dilate_data = mf.dilate()
    erode_data = mf.erode()
    open_data = mf.open()
    close_data = mf.close()
    
    
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.imshow(binary_data, cmap = cm.Greys_r)
    
    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111)
    ax2.imshow(dilate_data, cmap = cm.Greys_r)
    
    fig3 = plt.figure()
    ax3 = fig3.add_subplot(111)
    ax3.imshow(erode_data, cmap = cm.Greys_r)
    
    fig4 = plt.figure()
    ax4 = fig4.add_subplot(111)
    ax4.imshow(open_data, cmap = cm.Greys_r)
    
    fig5 = plt.figure()
    ax5 = fig5.add_subplot(111)
    ax5.imshow(close_data, cmap = cm.Greys_r)
    
    plt.show()