'''
Created on Oct 28, 2014

@author: daqing_yi
'''

from boundingShape import *
from sets import Set
import cv2

neighbor_operators = [ [1, 0], [1,-1], [0,-1], [-1,-1], [-1, 0], [-1, 1], [0, 1], [1, 1] ];

class ShapeDescriptor(object):

    def __init__(self, data, feature_num=15):
        self.data = data
        self.width = data.shape[0]
        self.height = data.shape[1]
        self.feature_num = feature_num
        self.label = None
        self.pixels = []
        for i in range(self.width):
            for j in range(self.height):
                if self.data[i,j] == 1:
                    self.pixels.append([i,j])
        
    def getFeatureVector(self):
        feature = np.zeros(self.feature_num, np.double)
        
        feature[0] = self.getCompactness()
        feature[1] = self.getRectangularity()
        feature[2] = self.getEccentricity()
        feature[3] = self.getElongation()
        feature[4] = self.getHoleAreaRatio()
        feature[5] = self.getSolidity()
        feature[6] = self.getConvexity()
        feature[7] = self.getEllipseRatio()
        
        '''
        mean = self.getMean()
        feature[4] = mean[0]
        feature[5] = mean[1]
        '''
        
        # Hu invariant moments
        contours, hierarchy = cv2.findContours(self.data, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        mom = cv2.moments(contours[0])
        mean = self.getMean()
        print "mean X " + str(mean[0]) + "  cv:"  + str(mom["m10"]/mom["m00"])
        print "mean Y " + str(mean[1]) + "  cv:"  + str(mom["m01"]/mom["m00"])
        var = self.getVar(mean)
        print "var 00 " + str(var[0,0]) + "  cv:"  + str(mom["mu20"]/mom["m00"])
        print "var 01 " + str(var[0,1]) + "  cv:"  + str(mom["mu11"]/mom["m00"])
        print "var 02 " + str(var[1,0]) + "  cv:"  + str(mom["mu11"]/mom["m00"])
        print "var 03 " + str(var[1,1]) + "  cv:"  + str(mom["mu02"]/mom["m00"])
        Humoments = cv2.HuMoments(mom)
        #print Humoments
        feature[8]  = Humoments[0]
        feature[9]  = Humoments[1]
        feature[10]  = Humoments[2]
        feature[11]  = Humoments[3]
        feature[12]  = Humoments[4]
        feature[13]  = Humoments[5]
        feature[14] = Humoments[6]
    
        return feature
        
    def findChainCode(self):        
        start = self.findChainCodeStart()
        cc = []
        chain = []
        dir = 0
        coord = [0, 0]
        coord[0], coord[1] = start[0], start[1]
        while True:
            newcoord = [0, 0]
            newcoord[0] = coord[0] + neighbor_operators[dir][0]
            newcoord[1] = coord[1] + neighbor_operators[dir][1]
            if True == self.isBoundaryPixel(newcoord[0], newcoord[1]):
                cc.append(dir)
                chain.append(newcoord)
                coord[0] = newcoord[0]
                coord[1] = newcoord[1]
                dir = (dir+2) % 8
                #print chain
            else:
                dir = (dir-1) % 8
            if coord[0]==start[0] and coord[1]==start[1]:
                break
            
        return cc, chain
        
    def findChainCodeStart(self):
        
        for i in range(self.width):
            for j in range(self.height):
                if self.isBoundaryPixel(i, j) == True:
                    return [i, j]
        return None
    
    def getBoundaryPixel(self):
        boundaryPixels = []
        for i in range(self.width):
            for j in range(self.height):
                if self.isBoundaryPixel(i, j) == True:
                    boundaryPixels.append( [i, j] )
        return boundaryPixels
    
    def getConvexity(self):
        contours, hierarchy = cv2.findContours(self.data, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=cv2.contourArea,reverse=True)
        cont_perimeter = cv2.arcLength(contours[0], closed=True)
        hull = cv2.convexHull(contours[0])
        hull_perimeter = cv2.arcLength(hull, closed=True)
        return float(cont_perimeter)/hull_perimeter
        
        
    def isBoundaryPixel(self, i, j):
        # val = 1 means a ON pixel
        boundary = False
        if self.data[i,j] == 1:
            for no in neighbor_operators:
                nx = i + no[0]
                ny = j + no[1]
                if nx < 0 or nx >= self.width or ny < 0 or ny >= self.height:
                    return False
                if self.data[nx, ny] == 0:
                    boundary = True
        return boundary
        
    def getPerimeter(self):
        cc, chain = self.findChainCode()
        length = 0.0
        for c in cc:
            if c%2==0:
                length += 1.0
            else:
                length += 1.414
                
        contours, hierarchy = cv2.findContours(self.data, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        arc_length = sum([cv2.arcLength(c,closed=True) for c in contours])
        print "Arc Length " + str(length) + " - CV2: " + str(arc_length)
        return length
    
    def getArea(self):
        area = 0.0
        for i in range(self.width):
            for j in range(self.height):
                if self.data[i,j]==1:
                    area += 1.0
        return area
    
    def getSolidity(self):
        contours, hierarchy = cv2.findContours(self.data, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=cv2.contourArea,reverse=True)
        cont_area = cv2.contourArea(contours[0])
        hull = cv2.convexHull(contours[0])
        hull_area = cv2.contourArea(hull)
        return float(cont_area)/hull_area
        
    def getHoleAreaRatio(self):
        contours, hierarchy = cv2.findContours(self.data, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=cv2.contourArea,reverse=True)
        cont_area = cv2.contourArea(contours[0])
        object_area = self.getArea()
        return float(object_area)/cont_area
    
    def getEllipseRatio(self):
        contours, hierarchy = cv2.findContours(self.data, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=cv2.contourArea,reverse=True)
        cont_area = cv2.contourArea(contours[0])
        
        cv2_rect = cv2.minAreaRect(contours[0])
        cv2_rect_width = int(cv2_rect[1][0])
        cv2_rect_height = int(cv2_rect[1][1])
        ellipse_area = cv2_rect_width * cv2_rect_height * np.pi
        
        return ellipse_area/cont_area
        
        
    def getCompactness(self):
        perimeter = self.getPerimeter()
        area = self.getArea()
        return (perimeter**2)/area
    
    def getRectWidthHeight(self):
        rect_box = self.getMinimumBoundingRectangle()
        rect_width = np.sqrt((rect_box[0][0] - rect_box[1][0])**2+(rect_box[0][1] - rect_box[1][1])**2) + 2
        rect_height = np.sqrt((rect_box[1][0] - rect_box[2][0])**2+(rect_box[1][1] - rect_box[2][1])**2) + 2
        
        contours, hierarchy = cv2.findContours(self.data, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=cv2.contourArea,reverse=True)
        cv2_rect = cv2.minAreaRect(contours[0])
        cv2_rect_width = int(cv2_rect[1][0])
        cv2_rect_height = int(cv2_rect[1][1])
        
        rW = np.min([rect_width, rect_height])
        rH = np.max([rect_width, rect_height])
        
        cv2_rW = np.min([cv2_rect_width, cv2_rect_height])
        cv2_rH = np.max([cv2_rect_width, cv2_rect_height])
         
        print "Rect W:" + str(int(rW)) + " H:" + str(int(rH)) + " - CV2: W:" + str(int(cv2_rW)) + " H:" + str(int(cv2_rH)) 
        return rW, rH
    

        
    
    def getRectangularity(self):
        rect_width, rect_height = self.getRectWidthHeight()
        return float(self.getArea())/(rect_width * rect_height)
        
    def getEccentricity(self):
        rect_width, rect_height = self.getRectWidthHeight()
        ratio = float(rect_width)/rect_height
        return ratio
    
    def getMean(self):
        meanVal = np.zeros(2, np.float)
        cnt = 0
        for i in range(self.width):
            for j in range(self.height):
                if self.data[i,j] == 1:
                    cnt += 1
                    meanVal[0] += i
                    meanVal[1] += j
        meanVal = meanVal / cnt      
        return meanVal
    
    def getVar(self, mean):
        var = np.zeros((2,2), np.float)
        mu_x = 0.0
        mu_y = 0.0
        mu_xy = 0.0
        cnt = 0
        for i in range(self.width):
            for j in range(self.height):
                if self.data[i,j] == 1:
                    cnt += 1
                    mu_x += (i-mean[0])**2
                    mu_y += (j-mean[1])**2
                    mu_xy += (i-mean[0])*(j-mean[1])
        mu_x = mu_x / cnt
        mu_y = mu_y / cnt
        mu_xy = mu_xy / cnt
        var[0,0] = mu_x
        var[1,1] = mu_y
        var[0,1] = var[1,0] = mu_xy
        
        return var       
                
    def getElongation(self):
        return 1.0 - self.getEccentricity()
    
    def getPsi_s_curve(self):
        return 0.0
    
    def getHorizontalProfile(self):
        profile = np.zeros(self.height, np.int)
        for j in range(self.height):
            for i in range(self.width):
                if self.data[i,j]==1:
                    profile[j] += 1
        return profile
    
    def getVerticalProfile(self):
        profile = np.zeros(self.width, np.int)
        for i in range(self.width):
            for j in range(self.width):
                if self.data[i,j]==1:
                    profile[i] += 1
        return profile        
    
    def getHoles(self):
        return 0.0
    
    def getCorners(self):
        return 0.0
    
    def getConvexHull(self):        
        points = Set()
        for i in range(self.width):
            for j in range(self.width):
                if self.data[i,j]==1:
                    points.add((i,j))
        convexHulls = getConvexHull(points)
        return convexHulls
    
    def getMinimumBoundingRectangle(self):
        points = Set()
        for i in range(self.width):
            for j in range(self.width):
                if self.data[i,j]==1:
                    points.add((i,j))
        info = getMinimumBoundingBox(points)
        return info[5]