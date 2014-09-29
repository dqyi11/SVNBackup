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

img_filename = 'circles.ppm'

detect_radius = 32

img = Image.open(img_filename).convert("L")
img = np.array(img)

img_hough = readFromCsv('img_hough.csv')

fig3 = plt.figure()
ax3 = fig3.add_subplot(111)
ax3.imshow(255*img_hough,cmap = 'gray')
ax3.set_title('Hough')
ax3.set_xticks([])
ax3.set_yticks([])

#writeToCsv('img_hough.csv', img_hough)

img_hough_sup = readFromCsv('img_hough_sup.csv')

fig4 = plt.figure()
ax4 = fig4.add_subplot(111)
ax4.imshow(255*img_hough_sup,cmap = 'gray')
ax4.set_title('Hough suppressed')
ax4.set_xticks([])
ax4.set_yticks([])

#writeToCsv('img_hough_sup.csv', img_hough_sup)

#plt.show()

plt.show(block=False)

centers = findByThreshold(img_hough_sup, 0.06)
#print centers

cimg = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)
for c in centers:
    # draw the outer circle
    cv2.circle(cimg,(c[0]-detect_radius,c[1]-detect_radius),detect_radius,(0,255,0),2)
    # draw the center of the circle
    cv2.circle(cimg,(c[0]-detect_radius,c[1]-detect_radius),2,(0,0,255),3)
    
    print str((c[0]-detect_radius,c[1]-detect_radius))

cv2.imshow('detected circles',cimg)
cv2.imwrite(img_filename+"-testHF02.png", cimg)
cv2.waitKey(0)

