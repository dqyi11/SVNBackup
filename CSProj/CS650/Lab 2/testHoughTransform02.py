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
from utilities import *

#img_filename = '2D_White_Box.pgm'
#img_filename = 'blocks.pgm'
#img_filename = 'simplecircles.ppm'
#img_filename = 'circles_s.ppm'
img_filename = 'circles.ppm'
#img_filename = 'coins.png'

detect_radius = 32

img = Image.open(img_filename).convert("L")
img = np.array(img)

#img_gauss = gaussianFilter(img)
img_gauss = cv2.medianBlur(img, 3)

#img_edge = MarrHildreth(img_gauss, 50)
#img_edge = canny(img_gauss, 20, 40)
img_edge = cv2.Canny(img_gauss, 60, 140)

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

img_hough = houghCircle(img_edge, [detect_radius])[:,:,0]

#print centers

fig3 = plt.figure()
ax3 = fig3.add_subplot(111)
ax3.imshow(255*img_hough,cmap = 'gray')
ax3.set_title('Hough')
ax3.set_xticks([])
ax3.set_yticks([])

plt.show()

'''

print "generate hough circles"

vg = houghCircleVariant(img_edge, [radius])
img_hough = vg.dumpHoughImgByRadusIndex(0)




fig3 = plt.figure()
ax3 = fig3.add_subplot(111)
ax3.imshow(255*img_hough,cmap = 'gray')
ax3.set_title('Hough')
ax3.set_xticks([])
ax3.set_yticks([])

writeToCsv(img_filename + '-img_hough.csv', img_hough)

for i in range(40):
    #print "revoting " + str(i)
    vg.weigthedRevote()

img_hough_sup = vg.dumpHoughImgByRadusIndex(0)

fig4 = plt.figure()
ax4 = fig4.add_subplot(111)
ax4.imshow(255*img_hough_sup,cmap = 'gray')
ax4.set_title('Hough suppressed')
ax4.set_xticks([])
ax4.set_yticks([])

writeToCsv(img_filename + '-img_hough_sup.csv', img_hough_sup)

plt.show(block=True)

'''

centers = findByThreshold(img_hough, 0.5)
#centers = findLocalMax(img_hough, 0.35)
#centers = findByThreshold(img_hough_sup, 0.5)
print centers

cimg = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)
for c in centers:
    # draw the outer circle
    cv2.circle(cimg,(c[0]-detect_radius,c[1]-detect_radius),detect_radius,(0,255,0),2)
    # draw the center of the circle
    cv2.circle(cimg,(c[0]-detect_radius,c[1]-detect_radius),2,(0,0,255),3)

cv2.imshow('detected circles',cimg)
cv2.imwrite(img_filename+'-detected.png',cimg)
cv2.waitKey(0)

