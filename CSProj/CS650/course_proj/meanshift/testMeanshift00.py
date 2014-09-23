'''
Created on Sep 23, 2014

@author: daqing_yi
'''

import cv2
from MeanShift import *

img_filename = 'DSC_5160.jpg'

img = cv2.imread(img_filename, cv2.IMREAD_COLOR)

img_filtered = meanShiftFilter(img, 10, 6.5)

cv2.imshow(img_filtered)
cv2.waitKey(0)