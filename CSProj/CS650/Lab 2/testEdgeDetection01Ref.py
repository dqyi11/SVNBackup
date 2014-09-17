'''
Created on Sep 17, 2014

@author: daqing_yi
'''

import cv2
import numpy as np
import matplotlib.pyplot as plt

img_filename = '2D_White_Box.pgm'
#img_filename = 'blocks.pgm'
img = cv2.imread(img_filename, 0)
        
img_laplacian = cv2.Laplacian(img,cv2.CV_64F)

fig = plt.figure()
ax1 = fig.add_subplot(121)
ax1.imshow(img,cmap = 'gray')
ax1.set_title('Original')
ax1.set_xticks([])
ax1.set_yticks([])

ax2 = fig.add_subplot(122)
ax2.imshow(img_laplacian,cmap = 'gray')
ax2.set_title('Laplacian')
ax2.set_xticks([])
ax2.set_yticks([])

plt.show()