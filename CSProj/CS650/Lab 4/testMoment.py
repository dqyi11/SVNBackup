'''
Created on 2014-11-3

@author: Walter
'''
import cv2

if __name__ == '__main__':
    
    img1_file = "shapes.pgm"
    img1 = cv2.imread(img1_file, 0)
    #print img1.shape

    #img1_bi = 1*(img1 >= 100)
    contours, hierarchy = cv2.findContours(img1, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    
    mom = cv2.moments(contours[0])
    
    Humoments = cv2.HuMoments(mom)
    print Humoments