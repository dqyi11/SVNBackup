'''
Created on 2014-11-20

@author: Walter
'''
import numpy as np
import cv2

def prematch1(img1, points1, img2, points2, window_size, threshold=0.3):
    new_points1 = []
    new_points2 = []
    for p1 in points1:
        for p2 in points2:
            new_points1.append([p1[0], p1[1]])
            new_points2.append([p2[0], p2[1]])
    return new_points1, new_points2

def getMean(img, p, window_size):
    vals = []
    for dx in range(-window_size, window_size+1):
        for dy in range(-window_size, window_size+1):
            if p[0]+dx >=0 and p[0]+dx < img.shape[0] and p[1]+dy >=0 and p[1]+dy < img.shape[1]:
                vals.append(img[p[0]+dx, p[1]+dy])
    return np.mean(np.array(vals))

def getStdDev(img, p, window_size):
    vals = []
    for dx in range(-window_size, window_size+1):
        for dy in range(-window_size, window_size+1):
            if p[0]+dx >=0 and p[0]+dx < img.shape[0] and p[1]+dy >=0 and p[1]+dy < img.shape[1]:
                vals.append(img[p[0]+dx, p[1]+dy])
                
                
    return np.std(np.array(vals))            
            
    

def getNCC(img1, p1, img2, p2, window_size):
    
    val1 = []
    val2 = []
    for dx in range(-window_size, window_size+1):
        for dy in range(-window_size, window_size+1):
            p1x = p1[0]+dx
            p1y = p1[1]+dy
            p2x = p2[0]+dx
            p2y = p2[1]+dy
            if p1x >=0 and p1x < img1.shape[1] and p1y >=0 and p1y < img1.shape[0] \
                and p2x >=0 and p2x < img2.shape[1] and p2y >=0 and p2y < img2.shape[0]:
                #print "p1 " + str(p1y) + "/" + str(img1.shape[0]) + " " + str(p1x) + "/" + str(img1.shape[1])
                #print "p2 " + str(p2y) + "/" + str(img2.shape[0]) + " " + str(p2x) + "/" + str(img2.shape[1])
                val1.append(img1[p1y,p1x])
                val2.append(img2[p2y,p2x])
                
    if len(val1) == 0:
        return 0.0
    
    mean1 = np.mean(val1)
    mean2 = np.mean(val2)
    std_dev1 = np.std(val1)
    std_dev2 = np.std(val2)
    
    sumVal = 0.0
    p_len = len(val1)
    for i in range(p_len):
        sumVal += (val1[i]-mean1)*(val2[i]-mean2)
    return float(sumVal)/((2*window_size+1)**2 * std_dev1 * std_dev2)


def prematch(img1, points1, img2, points2, window_size, threshold=0.3):
    new_points1 = []
    new_points2 = []

    p1_num = len(points1)
    p2_num = len(points2)
    ncc_vals = np.zeros((p1_num, p2_num), np.float)
    
    for i in range(p1_num):
        for j in range(p2_num):
            p1 = points1[i]
            p2 = points2[j]
            ncc = getNCC(img1, p1, img2, p2, window_size)
            if ncc != np.nan:
                ncc_vals[i,j] = ncc
            
    np.savetxt('NCC.txt', ncc_vals, fmt='%1.4f')
    
    '''
    for i in range(p1_num):
        for j in range(p2_num):
            if ncc_vals[i,j] >= threshold:
                new_points1.append(points1[i])
                new_points2.append(points2[j]) 
    '''
    for i in range(p1_num):
        max_idx = np.argmax(ncc_vals[i,:])
        max_val = ncc_vals[i, max_idx]
        #print "i: " + str(max_val)
        new_points1.append([points1[i][0], points1[i][1]])
        new_points2.append([points2[max_idx][0], points2[max_idx][1]])
        
    '''
    for j in range(p2_num):
        max_idx = np.argmax(ncc_vals[:,j])
        max_val = ncc_vals[max_idx, j]
        print "j: " + str(max_val)
        new_points1.append([points1[max_idx][0], points1[max_idx][1]])
        new_points2.append([points2[j][0], points2[j][1]])
    '''
        
    
    return new_points1, new_points2

def prematch2(img1, points1, img2, points2, window_size):
    
    sift = cv2.SIFT()
    kp1, des1 = sift.detectAndCompute(img1,None)
    kp2, des2 = sift.detectAndCompute(img2,None)
    
    # BFMatcher with default params
    bf = cv2.BFMatcher()
    matches = bf.knnMatch(des1,des2, k=2)
    
    # Apply ratio test
    newpoints1, newpoints2 = [], []
    for m,n in matches:
        if m.distance < 0.1*n.distance:
            newpoints1.append(kp1[m.queryIdx].pt)
            newpoints2.append(kp2[m.trainIdx].pt)
    return newpoints1, newpoints2
                
                
                    
                    
                    