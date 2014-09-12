'''
Created on Sep 12, 2014

@author: daqing_yi
'''

from PIL import Image
from skimage.filter import threshold_otsu, rank
import numpy as np

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
    
    threshold = threshold_otsu(img_data)
    
    
    print threshold
    
    

