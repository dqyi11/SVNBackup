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

detect_radius = 32

img = Image.open(img_filename).convert("L")
img = np.array(img)

#img_gauss = gaussianFilter(img)
img_gauss = copy.deepcopy(img)

img_edge = canny(img_gauss, 40, 80)
#img_edge = MarrHildreth(img_gauss, 50)

img_hough = houghCircle(img_edge, [detect_radius])[:,:,0]


for threshold in np.arange(0.8, 0.2, -0.1):
    centers = findLocalMax(img_hough, threshold)
    print threshold
    if len(centers) > 0:
        break

#print centers

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

#plt.show()

plt.show(block=False)

cimg = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)
for c in centers:
    cv2.circle(cimg,(c[0]-detect_radius,c[1]-detect_radius),detect_radius,(0,255,0),2)
    cv2.circle(cimg,(c[0]-detect_radius,c[1]-detect_radius),2,(0,0,255),3)
    
    print str((c[0]-detect_radius,c[1]-detect_radius))

cv2.imshow('detected circles',cimg)
cv2.imwrite(img_filename+"-hough01.png", cimg)
cv2.waitKey(0)

