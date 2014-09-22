'''
Created on Sep 17, 2014

@author: daqing_yi
'''

import cv2
import numpy as np
import matplotlib.pyplot as plt

img_filename = '2D_White_Box.pgm'
#img_filename = 'blocks.pgm'
#img_filename = 'simplecircles.ppm'
#img_filename = 'circles.ppm'
img = cv2.imread(img_filename, 0)

img_g_x = cv2.Sobel(img, cv2.CV_64F, 1, 0, ksize=3)
img_g_y = cv2.Sobel(img, cv2.CV_64F, 0, 1, ksize=3)

img_gm = np.zeros(img.shape, np.float)
img_go = np.zeros(img.shape, np.float)

for i in range(img.shape[0]):
    for j in range(img.shape[1]):
        img_gm[i,j] = np.sqrt((img_g_x[i,j])**2 + (img_g_y[i,j])**2)
        img_go[i,j] = np.arctan2(img_g_y[i,j], img_g_x[i,j])        


fig = plt.figure()
ax1 = fig.add_subplot(131)
ax1.imshow(img,cmap = 'gray')
ax1.set_title('Original')
ax1.set_xticks([])
ax1.set_yticks([])

ax2 = fig.add_subplot(132)
ax2.imshow(img_gm,cmap = 'gray')
ax2.set_title('gradient magnitude')
ax2.set_xticks([])
ax2.set_yticks([])

ax3 = fig.add_subplot(133)
ax3.imshow(img_go,cmap = 'gray')
ax3.set_title('gradient orientation')
ax3.set_xticks([])
ax3.set_yticks([])

plt.show()