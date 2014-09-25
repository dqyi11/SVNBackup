'''
Created on Sep 17, 2014

@author: daqing_yi
'''

from EdgeDetection import *
from PIL import Image
import numpy as np
import matplotlib.pyplot as plt


img_filename = '2D_White_Box.pgm'
#img_filename = 'blocks.pgm'
#img_filename = 'simplecircles.ppm'
#img_filename = 'circles.ppm'
img = Image.open(img_filename).convert("L")
img = np.array(img)

img_gm, img_go = sobel(img)

fig1 = plt.figure()
ax1 = fig1.add_subplot(111)
ax1.imshow(img,cmap = 'gray')
ax1.set_title('Original')
ax1.set_xticks([])
ax1.set_yticks([])

fig2 = plt.figure()
ax2 = fig2.add_subplot(111)
ax2.imshow(img_gm,cmap = 'gray')
ax2.set_title('gradient magnitude')
ax2.set_xticks([])
ax2.set_yticks([])

fig3 = plt.figure()
ax3 = fig3.add_subplot(111)
ax3.imshow(img_go,cmap = 'gray')
ax3.set_title('gradient orientation')
ax3.set_xticks([])
ax3.set_yticks([])

plt.show()