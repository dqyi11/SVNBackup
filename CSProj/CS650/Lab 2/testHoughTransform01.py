'''
Created on Sep 18, 2014

@author: daqing_yi
'''

from EdgeDetection import *
from HoughTransform import *
from VoteGame import *
from PIL import Image
import numpy as np
import matplotlib.pyplot as plt
import cv2

#img_filename = '2D_White_Box.pgm'
#img_filename = 'blocks.pgm'
img_filename = 'simplecircles.ppm'
#img_filename = 'circles.ppm'
#img_filename = 'coins.jpg'
#img_filename = 'circle_1.png'

img = Image.open(img_filename).convert("L")
img = np.array(img)

img_gauss = copy.deepcopy(img)
#img_gauss = gaussianFilter(img)

img_edge = canny(img_gauss, 40, 80)

img_hough = houghCircle(img_edge, [32])[:,:,0]


img_hough_gm, img_hough_go = sobel(img_hough)
img_hough_sup = nonmaximalSuppresion(img_hough, img_hough_go)


centers = findByThreshold(img_hough_sup*255, 100)
print centers

fig1 = plt.figure()
ax1 = fig1.add_subplot(111)
ax1.imshow(img,cmap = 'gray')
ax1.set_title('Original')
ax1.set_xticks([])
ax1.set_yticks([])

fig2 = plt.figure()
ax2 = fig2.add_subplot(111)
ax2.imshow(img_edge,cmap = 'gray')
ax2.set_title('Canny')
ax2.set_xticks([])
ax2.set_yticks([])

fig3 = plt.figure()
ax3 = fig3.add_subplot(111)
ax3.imshow(255*img_hough,cmap = 'gray')
ax3.set_title('Hough')
ax3.set_xticks([])
ax3.set_yticks([])

fig4 = plt.figure()
ax4 = fig4.add_subplot(111)
ax4.imshow(img_hough_sup,cmap = 'gray')
ax4.set_title('Hough suppressed')
ax4.set_xticks([])
ax4.set_yticks([])

plt.show(block=False)

cimg = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)
for c in centers:
    cv2.circle(cimg,(c[0]-32,c[1]-32),32,(0,255,0),2)
    cv2.circle(cimg,(c[0]-32,c[1]-32),2,(0,0,255),3)

cv2.imshow('detected circles',cimg)
cv2.imwrite(img_filename+"-hough01.png", cimg)
cv2.waitKey(0)
