'''
Created on Sep 18, 2014

@author: daqing_yi
'''

from EdgeDetection import *
from HoughTransform import *
from PIL import Image
import numpy as np
import matplotlib.pyplot as plt


#img_filename = '2D_White_Box.pgm'
#img_filename = 'blocks.pgm'
#img_filename = 'simplecircles.ppm'
img_filename = 'circles.ppm'
img = Image.open(img_filename).convert("L")
img = np.array(img)

img_canny = canny(img)

img_hough = houghCircle(img_canny, 32)
img_hough_max = np.max(np.max(img_hough))
img_hough_norm = img_hough / float(img_hough_max)

fig = plt.figure()
ax1 = fig.add_subplot(131)
ax1.imshow(img,cmap = 'gray')
ax1.set_title('Original')
ax1.set_xticks([])
ax1.set_yticks([])

ax2 = fig.add_subplot(132)
ax2.imshow(img_canny,cmap = 'gray')
ax2.set_title('Canny')
ax2.set_xticks([])
ax2.set_yticks([])

ax3 = fig.add_subplot(133)
ax3.imshow(img_hough_norm,cmap = 'gray')
ax3.set_title('Hough')
ax3.set_xticks([])
ax3.set_yticks([])

plt.show()
