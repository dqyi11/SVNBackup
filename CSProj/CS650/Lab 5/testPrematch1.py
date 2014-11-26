'''
Created on Nov 24, 2014

@author: daqing_yi
'''

import cv2
from HarrisCornerDetector import *
from NCCMatching import *

if __name__ == '__main__':
    
    #img = cv2.imread('chessboard2.jpg', 0)
    filename1 = 'IMG_1341_s'
    filename2 = 'IMG_1343_s'
    img1 = cv2.imread(filename1+'.JPG', 0)
    img2 = cv2.imread(filename2+'.JPG', 0)
    
    newpoints1, newpoints2 = prematch2(img1, [], img2, [], 3)
    
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
        print str(p1) + " -- " + str(p2)
        cv2.circle(cimg1,(int(p1[0])-point_radius,int(p1[1])-point_radius),point_radius,rndColors[i],point_radius)
        cv2.circle(cimg2,(int(p2[0])-point_radius,int(p2[1])-point_radius),point_radius,rndColors[i],point_radius)
    #cv2.imshow(filename1, cimg1)
    cv2.imwrite(filename1+"_l.jpg", cimg1)
    #cv2.imshow(filename2, cimg2)
    cv2.imwrite(filename2+"_l.jpg", cimg2)
    #cv2.waitKey(0)

    
        