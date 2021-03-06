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
#img_filename = 'simplecircles.ppm'
img_filename = 'circles.ppm'
#img_filename = 'coins.jpg'
#img_filename = 'circle_1.png'

detect_radius = 48

img = cv2.imread(img_filename,0)
img = cv2.medianBlur(img,5)
img = np.array(img)

#img_gauss = gaussianFilter(img)
img_gauss = copy.deepcopy(img)

img_edge = canny(img_gauss, 40, 80)
#img_edge = MarrHildreth(img_gauss, 60)

img_hough = houghCircle(img_edge, [detect_radius])[:,:,0]

for threshold in np.arange(0.8, 0.2, -0.1):
    centers = findLocalMax(img_hough, threshold)
    print threshold
    if len(centers) > 1:
        break

print centers

fig = plt.figure()
ax1 = fig.add_subplot(221)
ax1.imshow(img,cmap = 'gray')
ax1.set_title('Original')
ax1.set_xticks([])
ax1.set_yticks([])

ax2 = fig.add_subplot(222)
ax2.imshow(img_edge,cmap = 'gray')
ax2.set_title('Canny')
ax2.set_xticks([])
ax2.set_yticks([])

ax3 = fig.add_subplot(223)
ax3.imshow(255*img_hough,cmap = 'gray')
ax3.set_title('Hough')
ax3.set_xticks([])
ax3.set_yticks([])


plt.show(block=False)

cimg = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)
for c in centers:
    cv2.circle(cimg,(c[0]-detect_radius,c[1]-detect_radius),detect_radius,(0,255,0),2)
    cv2.circle(cimg,(c[0]-detect_radius,c[1]-detect_radius),2,(0,0,255),3)


cv2.imshow('detected circles',cimg)
cv2.waitKey(0)

