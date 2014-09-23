'''
Created on Sep 18, 2014

@author: daqing_yi
'''

from EdgeDetection import *
from HoughTransform import *
from PIL import Image
import numpy as np
import matplotlib.pyplot as plt
import cv2

#img_filename = '2D_White_Box.pgm'
#img_filename = 'blocks.pgm'
img_filename = 'simplecircles.ppm'
img_filename = 'circles.ppm'
img_filename = 'coins.jpg'

img = Image.open(img_filename).convert("L")
img = np.array(img)

img_gauss = gaussianFilter(img)

img_canny = canny(img_gauss, 70, 150)

img_hough = houghCircle(img_canny, 32)
img_hough_max = np.max(np.max(img_hough))
img_hough_norm = img_hough / float(img_hough_max)

img_hough_gm, img_hough_go = sobel(img_hough)
img_hough_sup = nonmaximalSuppresion(img_hough_norm, img_hough_go)

centers = findLocalMax(img_hough_sup)
print centers

fig = plt.figure()
ax1 = fig.add_subplot(141)
ax1.imshow(img,cmap = 'gray')
ax1.set_title('Original')
ax1.set_xticks([])
ax1.set_yticks([])

ax2 = fig.add_subplot(142)
ax2.imshow(img_canny,cmap = 'gray')
ax2.set_title('Canny')
ax2.set_xticks([])
ax2.set_yticks([])

ax3 = fig.add_subplot(143)
ax3.imshow(255*img_hough_norm,cmap = 'gray')
ax3.set_title('Hough')
ax3.set_xticks([])
ax3.set_yticks([])

ax4 = fig.add_subplot(144)
ax4.imshow(img_hough_sup,cmap = 'gray')
ax4.set_title('Hough suppressed')
ax4.set_xticks([])
ax4.set_yticks([])

plt.show()

cimg = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)
for c in centers:
    # draw the outer circle
    cv2.circle(cimg,(c[0],c[1]),32,(0,255,0),2)
    # draw the center of the circle
    cv2.circle(cimg,(c[0],c[1]),2,(0,0,255),3)

cv2.imshow('detected circles',cimg)
cv2.waitKey(0)
