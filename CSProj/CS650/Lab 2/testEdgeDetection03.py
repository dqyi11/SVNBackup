'''
Created on 2014-9-21

@author: Walter
'''

from EdgeDetection import *
from PIL import Image
import numpy as np
import matplotlib.pyplot as plt
import copy

#img_filename = '2D_White_Box.pgm'
img_filename = 'blocks.pgm'
#img_filename = 'simplecircles.ppm'
#img_filename = 'circles.ppm'
#img_filename = 'coins.png'
img = np.array(Image.open(img_filename).convert("L"))

img_gauss = gaussianFilter(img)
#img_gauss = copy.deepcopy(img)

img_mh = MarrHildreth(img_gauss,40)

fig1 = plt.figure()
ax1 = fig1.add_subplot(111)
ax1.imshow(img,cmap = 'gray')
ax1.set_title('Original')
ax1.set_xticks([])
ax1.set_yticks([])

fig2 = plt.figure()
ax2 = fig2.add_subplot(111)
ax2.imshow(img_mh,cmap = 'gray')
ax2.set_title('MarrHildreth')
ax2.set_xticks([])
ax2.set_yticks([])

plt.show()