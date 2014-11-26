'''
Created on 2014-11-19

@author: Walter
'''

from HarrisCornerDetector import *
import cv2
import matplotlib.pyplot as plt
import numpy as np


if __name__ == '__main__':
    
    #img = cv2.imread('chessboard2.jpg', 0)
    filename = 'IMG_1341'
    #filename = 'IMG_1342.JPG'
    img = cv2.imread(filename+".JPG", 0)
    print img.shape
    hcd = HarrisCornerDetector(5 ,0.2, 100)
    points = hcd.getCornerPoints(img)
    
    print len(points)
    print points
    
    point_radius = 20
    cimg = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)
    for p in points:
        cv2.circle(cimg,(p[0]-point_radius,p[1]-point_radius),point_radius,(255,0,0),point_radius)
    cv2.imwrite(filename+"_hcd.jpg", cimg)
    #cv2.waitKey(0)
    
    
    