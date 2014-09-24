'''
Created on Sep 17, 2014

@author: daqing_yi
'''

from EdgeDetection import *
from PIL import Image
import numpy as np
import matplotlib.pyplot as plt
import copy

img_filename = '2D_White_Box.pgm'
#img_filename = 'blocks.pgm'
#img_filename = 'simplecircles.ppm'
#img_filename = 'circles.ppm'
img_filename = 'coins.png'
img = np.array(Image.open(img_filename).convert("L"))

img_gauss = gaussianFilter(img)
#img_gauss = copy.deepcopy(img)

img_canny = canny(img_gauss, 30, 100)

fig = plt.figure()
ax1 = fig.add_subplot(121)
ax1.imshow(img,cmap = 'gray')
ax1.set_title('Original')
ax1.set_xticks([])
ax1.set_yticks([])

ax2 = fig.add_subplot(122)
ax2.imshow(img_canny,cmap = 'gray')
ax2.set_title('Canny')
ax2.set_xticks([])
ax2.set_yticks([])

plt.show()