'''
Created on Nov 20, 2014

@author: daqing_yi
'''

from HarrisCornerDetector import *
from NCCMatching import *
import cv2
import matplotlib.pyplot as plt
import numpy as np
import NCCMatching
from ImageManager import *

if __name__ == '__main__':
    
    #img = cv2.imread('chessboard2.jpg', 0)
    filename1 = 'IMG_1341'
    filename2 = 'IMG_1342'
    img1 = cv2.imread(filename1+'.JPG', 0)
    img2 = cv2.imread(filename2+'.JPG', 0)
    hcd = HarrisCornerDetector(3 ,0.2, 100)
    points1 = hcd.getCornerPoints(img1)
    points2 = hcd.getCornerPoints(img2)
    
    newpoints1, newpoints2 = prematch(img1, points1, img2, points2, 10)
    
    points_num = len(newpoints1)
    rndColors = []
    for i in range(points_num):
        rndVals = np.random.randint(0, 255,3)
        rndColors.append( (rndVals[0], rndVals[1], rndVals[2]) )
            
    point_radius = 20
    cimg1 = cv2.cvtColor(img1,cv2.COLOR_GRAY2BGR)
    cimg2 = cv2.cvtColor(img2,cv2.COLOR_GRAY2BGR)
    for i in range(points_num):
        p1 = newpoints1[i]
        p2 = newpoints2[i]
        #print str(p1) + " -- " + str(p2)
        cv2.circle(cimg1,(p1[0]-point_radius,p1[1]-point_radius),point_radius,rndColors[i],point_radius)
        cv2.circle(cimg2,(p2[0]-point_radius,p2[1]-point_radius),point_radius,rndColors[i],point_radius)
    #cv2.imshow(filename1, cimg1)
    cv2.imwrite(filename1+"_l.jpg", cimg1)
    #cv2.imshow(filename2, cimg2)
    cv2.imwrite(filename2+"_l.jpg", cimg2)
    #cv2.waitKey(0)
    
    print "matched len " + str(len(newpoints1))
    
    numiters = 2000
    im = ImageManager(None)
    H, best_set =  im.ransac2(img1, newpoints1, img2, newpoints2, numiters, 20)
    #H = self.ransac(matchedpoints1, matchedpoints2, numiters)
    print H
    
    points_num = len(best_set)
    rndColors = []
    for i in range(points_num):
        rndVals = np.random.randint(0, 255,3)
        rndColors.append( (rndVals[0], rndVals[1], rndVals[2]) )
        
    point_radius = 20
    cimg1 = cv2.cvtColor(img1,cv2.COLOR_GRAY2BGR)
    cimg2 = cv2.cvtColor(img2,cv2.COLOR_GRAY2BGR)
    for i in range(points_num):
        #print best_set[i]
        p1 = best_set[i][0]
        p2 = best_set[i][1]
        print str(p1) + " -- " + str(p2)
        cv2.circle(cimg1,(p1[0]-point_radius,p1[1]-point_radius),point_radius,rndColors[i],point_radius)
        cv2.circle(cimg2,(p2[0]-point_radius,p2[1]-point_radius),point_radius,rndColors[i],point_radius)
    #cv2.imshow(filename1, cimg1)
    cv2.imwrite(filename1+"_l_best.jpg", cimg1)
    #cv2.imshow(filename2, cimg2)
    cv2.imwrite(filename2+"_l_best.jpg", cimg2)
    
    '''
    ps1 = []
    ps2 = []
    for p in best_set:
        ps1.append(p[0])
        ps2.append(p[1])
    '''