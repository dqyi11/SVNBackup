'''
Created on Sep 23, 2014

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
img_filename = 'circles.ppm'
#img_filename = 'coins.png'

img = Image.open(img_filename).convert("L")
img = np.array(img)

img_gauss = gaussianFilter(img)

img_edge = MarrHildreth(img_gauss, 50)

print "generate hough circles"

vg = houghCircleVariant(img_edge, [32])

img_hough = vg.dumpHoughImgByRadusIndex(0)

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

for i in range(4):
    print "revoting " + str(i)
    vg.weigthedRevote()

img_hough_sup = vg.dumpHoughImgByRadusIndex(0)

fig4 = plt.figure()
ax4 = fig4.add_subplot(224)
ax4.imshow(255*img_hough_sup,cmap = 'gray')
ax4.set_title('Hough suppressed')
ax4.set_xticks([])
ax4.set_yticks([])

plt.show(block=True)

'''
cimg = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)
for c in centers:
    # draw the outer circle
    cv2.circle(cimg,(c[0],c[1]),32,(0,255,0),2)
    # draw the center of the circle
    #cv2.circle(cimg,(c[0],c[1]),2,(0,0,255),3)

cv2.imshow('detected circles',cimg)
cv2.waitKey(0)
'''