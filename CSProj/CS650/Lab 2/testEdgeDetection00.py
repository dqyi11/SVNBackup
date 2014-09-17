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
img = Image.open(img_filename).convert("L")
img = np.array(img)

img_gm, img_go = sobel(img)
#img_laplacian = laplacian(img)

plt.subplot(1,3,1),plt.imshow(img,cmap = 'gray')
plt.title('Original'), plt.xticks([]), plt.yticks([])
plt.subplot(1,3,2),plt.imshow(img_gm,cmap = 'gray')
plt.title('gradient magnitude'), plt.xticks([]), plt.yticks([])
plt.subplot(1,3,3),plt.imshow(img_go,cmap = 'gray')
plt.title('gradient orientation'), plt.xticks([]), plt.yticks([])

plt.show()