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
from utilities import *

if __name__ == '__main__':
    
    img_filename = 'Morphology_1.png'
    img = Image.open(img_filename).convert("L")

    img_width = img.size[0]
    img_height = img.size[1]
    #print "W:"+str(img_width) + " H:" + str(img_height)
    
    img_data = np.array(img)
    
    #print img_data.shape

    hist, bin_edges = np.histogram(img_data, np.arange(256))
    
    #print hist
    threshold, intraclass_variances = otsu(hist, img_width*img_height)
    
    print "Threshold: " + str(threshold)    
    
    binary_data = binarize(img_data, threshold)
    
    maskSize = [13,13]
    mf = NorphologicalFiltering(binary_data, maskSize)
    
    
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_title('Binary')
    ax.imshow(binary_data, cmap = cm.Greys_r)
    
    dilate_data = mf.dilate()
    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111)
    ax2.set_title('Dilate ' + str(mf.maskSize))
    ax2.imshow(dilate_data, cmap = cm.Greys_r)
    writeToCsv(img_filename+'.dilate.csv', dilate_data)
    
    erode_data = mf.erode()
    fig3 = plt.figure()
    ax3 = fig3.add_subplot(111)
    ax3.set_title('Erode '  + str(mf.maskSize))
    ax3.imshow(erode_data, cmap = cm.Greys_r)
    writeToCsv(img_filename+'.erode.csv', erode_data)
    
    
    open_data = mf.dilate(erode_data) #mf.open()
    fig4 = plt.figure()
    ax4 = fig4.add_subplot(111)
    ax4.set_title('Open ' + str(mf.maskSize))
    ax4.imshow(open_data, cmap = cm.Greys_r)
    writeToCsv(img_filename+'.open.csv', open_data)
    
    close_data = mf.erode(dilate_data) #mf.close()
    fig5 = plt.figure()
    ax5 = fig5.add_subplot(111)
    ax5.set_title('Close ' + str(mf.maskSize))
    ax5.imshow(close_data, cmap = cm.Greys_r)
    writeToCsv(img_filename+'.close.csv', close_data)    
    
    plt.show()