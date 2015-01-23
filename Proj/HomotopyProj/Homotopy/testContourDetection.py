'''
Created on Jan 23, 2015

@author: daqing_yi
'''

import numpy as np
import cv2
 
im = cv2.imread('map01.jpg')
imgray = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
imgray = 255-imgray
ret,thresh = cv2.threshold(imgray,127,255,0)

contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
#contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

print contours

cv2.drawContours(im,contours,-1,(0,255,0),3)
cv2.imshow('c', im)

cv2.waitKey(0)