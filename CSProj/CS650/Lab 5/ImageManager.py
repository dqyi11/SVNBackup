'''
Created on Nov 13, 2014

@author: hcmi
'''
from PyQt4 import QtGui, QtCore
from NCCMatching import *
from HarrisCornerDetector import *
import numpy as np
import sys
import cv2

class LabeledImage(object):
    
    def __init__(self, dict_entry):
        self.id = dict_entry["id"]
        self.location = dict_entry["location"]
        self.labeled_pixels = dict_entry["points"]
        self.img_array = cv2.imread(self.location, 0)
            
        self.pixmap = QtGui.QPixmap(self.location)
        self.name = QtCore.QFileInfo(self.location).fileName()
        
        self.labeled_id_list = []
        for p in self.labeled_pixels:
            self.labeled_id_list.append(p["id"])
            
        #print self.id
        #print self.location
        #print self.labeled_pixels
        
    def getPixelPos(self, idx):
        pos = [None, None]
        for p in self.labeled_pixels:
            if p["id"] == idx:
                pos[0] = p["x"]
                pos[1] = p["y"]
        return pos
        

class ImageManager(object):


    def __init__(self, json_dicts):
        if json_dicts != None:
            self.id_num = json_dicts["totalpointids"]
            self.image_num = len(json_dicts["pictures"])
            self.images = []
            for i in range(self.image_num):
                img = LabeledImage(json_dicts["pictures"][i])
                self.images.append(img)
                
            self.label_colors = []
            for i in range(self.id_num):
                rndVals = np.random.randint(0, 255,3)
                rndColor = QtGui.QColor(rndVals[0], rndVals[1], rndVals[2])
                self.label_colors.append(rndColor)
            
    
    def calcHomographyUsingIndex(self, idx_from, idx_to):
        from_img = self.images[idx_from]
        to_img = self.images[idx_to]
        
        join_labeled_id_list = self.getListJoin(from_img.labeled_id_list, to_img.labeled_id_list)
        feature_points_num = len(join_labeled_id_list)
        if feature_points_num < 4:
            return None
        
        feature_points_num = 4
        
        #print join_labeled_id_list
        
        from_points = []
        to_points = []
        for idx in join_labeled_id_list:
            #print "FROM ID " + str(id) + " " + str(from_img.getPixelPos(id))
            #print "TO ID " + str(id) + " " + str(to_img.getPixelPos(id))
            from_points.append(from_img.getPixelPos(idx))
            to_points.append(to_img.getPixelPos(idx))
            
        #print "From Points"
        #print from_points
        #print "To Points"
        #print to_points
        return self.calcHomograph(from_points, to_points)
        
    def calcHomograph(self, from_points, to_points):
        feature_points_num = 4
        H = np.ones((3,3), np.float)
        A = np.zeros((2*feature_points_num,8), np.float)
        to_f = np.zeros((2*feature_points_num,1),np.float)
        
        for i in range(feature_points_num):        
            A[i*2,0] = from_points[i][0]
            A[i*2,1] = from_points[i][1]
            A[i*2,2] = 1.0
            A[i*2,3] = 0.0
            A[i*2,4] = 0.0
            A[i*2,5] = 0.0
            A[i*2,6] = -to_points[i][0]*from_points[i][0]
            A[i*2,7] = -to_points[i][0]*from_points[i][1]
            
            A[i*2+1,0] = 0.0
            A[i*2+1,1] = 0.0
            A[i*2+1,2] = 0.0
            A[i*2+1,3] = from_points[i][0]
            A[i*2+1,4] = from_points[i][1]
            A[i*2+1,5] = 1.0
            A[i*2+1,6] = -to_points[i][1]*from_points[i][0]
            A[i*2+1,7] = -to_points[i][1]*from_points[i][1]
        
        for i in range(feature_points_num):
            to_f[2*i,0]   = to_points[i][0]
            to_f[2*i+1,0] = to_points[i][1]
        
        matrix_A = np.matrix(A)
        matrix_to_f = np.matrix(to_f)
        
        #print matrix_A.shape
        #print matrix_to_f.shape
        
        if np.linalg.matrix_rank(matrix_A) != matrix_A.shape[0]:
            return H
        
        matrix_h_f = np.linalg.inv(matrix_A).dot(matrix_to_f)
        
        h_f = np.array(matrix_h_f)
        
        H[0,0] = h_f[0,0]
        H[0,1] = h_f[1,0]
        H[0,2] = h_f[2,0]
        H[1,0] = h_f[3,0]
        H[1,1] = h_f[4,0]
        H[1,2] = h_f[5,0]
        H[2,0] = h_f[6,0]
        H[2,1] = h_f[7,0]
        
        '''
        #Verify the result
        ref_to_f = np.zeros((2*feature_points_num,1), np.float)
        for i in range(feature_points_num):
            ref_to_f[i*2,0] = to_points[i][0]
            ref_to_f[i*2+1,0] = to_points[i][1]
            
        
        to_f = np.zeros((8,1), np.float)
        for i in range(4):
            scale = float(H[2,0]*from_points[i][0]+H[2,1]*from_points[i][1]+1.0)
            to_f[i*2,0] = (H[0,0]*from_points[i][0]+H[0,1]*from_points[i][1]+H[0,2]) / scale
            to_f[i*2+1,0] = (H[1,0]*from_points[i][0]+H[1,1]*from_points[i][1]+H[1,2]) / scale
        
        err = to_f - ref_to_f
        #print to_f
        #print ref_to_f
        
        print "error " + str(err)
        '''
        return H
    
    def calcHomographyByRegressionUsingIndex(self, idx_from, idx_to):
        from_img = self.images[idx_from]
        to_img = self.images[idx_to]
        
        join_labeled_id_list = self.getListJoin(from_img.labeled_id_list, to_img.labeled_id_list)
        feature_points_num = len(join_labeled_id_list)
        if feature_points_num < 4:
            return None
        
        #feature_points_num = 4
        
        
        
        #print join_labeled_id_list
        
        from_points = []
        to_points = []
        for idx in join_labeled_id_list:
            #print "FROM ID " + str(id) + " " + str(from_img.getPixelPos(id))
            #print "TO ID " + str(id) + " " + str(to_img.getPixelPos(id))
            from_points.append(from_img.getPixelPos(idx))
            to_points.append(to_img.getPixelPos(idx))
        
        #print "From Points"
        #print from_points
        #print "To Points"
        #print to_points
        
        return self.calcHomographyByRegression(from_points, to_points)
        
    def calcHomographyByRegression(self, from_points, to_points):
        
        feature_points_num = len(from_points)
        
        H = np.ones((3,3), np.float)
        A = np.zeros((2*feature_points_num,8), np.float)
        to_f = np.zeros((2*feature_points_num,1),np.float)
        
        for i in range(feature_points_num):        
            A[i*2,0] = from_points[i][0]
            A[i*2,1] = from_points[i][1]
            A[i*2,2] = 1.0
            A[i*2,3] = 0.0
            A[i*2,4] = 0.0
            A[i*2,5] = 0.0
            A[i*2,6] = -to_points[i][0]*from_points[i][0]
            A[i*2,7] = -to_points[i][0]*from_points[i][1]
            
            A[i*2+1,0] = 0.0
            A[i*2+1,1] = 0.0
            A[i*2+1,2] = 0.0
            A[i*2+1,3] = from_points[i][0]
            A[i*2+1,4] = from_points[i][1]
            A[i*2+1,5] = 1.0
            A[i*2+1,6] = -to_points[i][1]*from_points[i][0]
            A[i*2+1,7] = -to_points[i][1]*from_points[i][1]
        
        for i in range(feature_points_num):
            to_f[2*i,0]   = to_points[i][0]
            to_f[2*i+1,0] = to_points[i][1]
        
        matrix_A = np.matrix(A)
        matrix_to_f = np.matrix(to_f)
        
        #print matrix_A.shape
        #print matrix_to_f.shape
        
        '''
        if np.linalg.matrix_rank(matrix_A) != matrix_A.shape[0]:
            print "not full rank == error"
            return H
        '''
        
        matrix_h_f = np.dot(np.dot(np.linalg.inv(np.dot(matrix_A.T, matrix_A)) , matrix_A.T) , matrix_to_f)
        #matrix_h_f = np.linalg.inv(matrix_A).dot(matrix_to_f)
        
        h_f = np.array(matrix_h_f)
        
        H[0,0] = h_f[0,0]
        H[0,1] = h_f[1,0]
        H[0,2] = h_f[2,0]
        H[1,0] = h_f[3,0]
        H[1,1] = h_f[4,0]
        H[1,2] = h_f[5,0]
        H[2,0] = h_f[6,0]
        H[2,1] = h_f[7,0]
        
        '''
        #Verify the result
        ref_to_f = np.zeros((2*feature_points_num,1), np.float)
        for i in range(feature_points_num):
            ref_to_f[i*2,0] = to_points[i][0]
            ref_to_f[i*2+1,0] = to_points[i][1]
            
        
        to_f = np.zeros((2*feature_points_num,1), np.float)
        for i in range(feature_points_num):
            scale = float(H[2,0]*from_points[i][0]+H[2,1]*from_points[i][1]+1.0)
            to_f[i*2,0] = (H[0,0]*from_points[i][0]+H[0,1]*from_points[i][1]+H[0,2]) / scale
            to_f[i*2+1,0] = (H[1,0]*from_points[i][0]+H[1,1]*from_points[i][1]+H[1,2]) / scale
        
        err = to_f - ref_to_f
        #print to_f
        #print ref_to_f
        
        print "error " + str(err)
        '''
        return H
    
    def getListJoin(self, list_a, list_b):
        join_list = []
        for a in list_a:
            if (a in list_b):
                join_list.append(a)
        return join_list

    def interpolate(self, pt1, pt2, pt3, pt4, xpercent, ypercent):
        return ((pt1 * (1-ypercent) + pt3 * ypercent) * (1-xpercent)) + ((pt2 * (1-ypercent) + pt4 * ypercent) * xpercent)
                
    #img = Old image that needs to be morphed
    #matrix = Homography matrix
    def bilinearInterpolation(self, img, matrix):
        sz1 = np.ones([3,1])
        sz2 = np.ones((3,1))
        sz3 = np.ones((3,1))
        sz4 = np.ones((3,1))
        
        sz1[0,0] = 0
        sz1[1,0] = 0
        sz2[0,0] = int(img.width())
        sz2[1,0] = 0
        sz3[0,0] = 0
        sz3[1,0] = int(img.height())
        sz4[0,0] = int(img.width())
        sz4[1,0] = int(img.height())
        
        newsize1 = np.dot(matrix,sz1)
        newsize2 = np.dot(matrix,sz2)
        newsize3 = np.dot(matrix,sz3)
        newsize4 = np.dot(matrix,sz4)
        
        #xlist = [newsize1[0,0], newsize2[0,0], newsize3[0,0], newsize4[0,0]]
        #ylist = [newsize1[1,0], newsize2[1,0], newsize3[1,0], newsize4[1,0]]
        xlist = [newsize1[0,0] / newsize1[2,0], newsize2[0,0] / newsize2[2,0], newsize3[0,0] / newsize3[2,0], newsize4[0,0] / newsize4[2,0]]
        ylist = [newsize1[1,0] / newsize1[2,0], newsize2[1,0] / newsize2[2,0], newsize3[1,0] / newsize3[2,0], newsize4[1,0] / newsize4[2,0]]
        transx = int(min(xlist))
        transy = int(min(ylist))

        newwidth = int(max(xlist)) - transx
        newheight = int(max(ylist)) - transy
        
        newimg = QtGui.QImage(newwidth, newheight, QtGui.QImage.Format_ARGB32)
        invmatrix = np.linalg.inv(matrix)
        rimg = np.zeros((img.width(), img.height(), 3), np.uint8)
        
        for y in range(img.height()):
            if y%50 == 0:
                sys.stdout.write("copying image line: " + str(y) + "\r")
                sys.stdout.flush()
            for x in range(img.width()):
                val = QtGui.QColor(img.pixel(x,y))
                rimg[x,y,0] = val.red()
                rimg[x,y,1] = val.green()
                rimg[x,y,2] = val.blue()
        
        sys.stdout.write("\n")
        
        for y in range(newheight):
            if y%50==0:
                sys.stdout.write("pos: " + str(y) + "\r")
                sys.stdout.flush()
            for x in range(newwidth):
                point = np.ones([3,1])
                point[0,0] = x + transx
                point[1,0] = y + transy
                pos = np.dot(invmatrix,point)
                posx = (pos[0,0] / pos[2,0])
                posy = (pos[1,0] / pos[2,0])
                
                val = QtGui.QColor()

                pt1pl = (np.floor(posx), np.floor(posy))
                pt2pl = (np.ceil(posx), np.floor(posy))
                pt3pl = (np.floor(posx), np.ceil(posy))
                pt4pl = (np.ceil(posx), np.ceil(posy))
                
                if (pt4pl[0] > img.width()-1 or pt4pl[1] > img.height()-1 or pt1pl[0] < 0 or pt1pl[1] < 0):
                    val = QtGui.QColor(255,255,255,0)
                else:                   
                    pt1n = rimg[pt1pl[0], pt1pl[1], :]
                    pt2n = rimg[pt2pl[0], pt2pl[1], :]
                    pt3n = rimg[pt3pl[0], pt3pl[1], :]
                    pt4n = rimg[pt4pl[0], pt4pl[1], :]
            
                    percentx = posx - np.floor(posx)
                    percenty = posy - np.floor(posy)
            
                    newvalr = self.interpolate(pt1n[0], pt2n[0], pt3n[0], pt4n[0], percentx, percenty)
                    newvalg = self.interpolate(pt1n[1], pt2n[1], pt3n[1], pt4n[1], percentx, percenty)
                    newvalb = self.interpolate(pt1n[2], pt2n[2], pt3n[2], pt4n[2], percentx, percenty)
            
                    val = QtGui.QColor(newvalr, newvalg, newvalb)


                newimg.setPixel(QtCore.QPoint(x, y), val.rgba())
        sys.stdout.write("\n")
        return newimg, (transx, transy)
    
    def calcHomographyByRANSACUsingIndex(self, idx_from, idx_to):
        from_img = self.images[idx_from]
        to_img = self.images[idx_to]
        
        join_labeled_id_list = self.getListJoin(from_img.labeled_id_list, to_img.labeled_id_list)
        feature_points_num = len(join_labeled_id_list)
        if feature_points_num < 4:
            return None
        
        #feature_points_num = 4
        
        
        
        #print join_labeled_id_list
        
        from_points = []
        to_points = []
        for idx in join_labeled_id_list:
            #print "FROM ID " + str(id) + " " + str(from_img.getPixelPos(id))
            #print "TO ID " + str(id) + " " + str(to_img.getPixelPos(id))
            from_points.append(from_img.getPixelPos(idx))
            to_points.append(to_img.getPixelPos(idx))
            
        return self.ransac(from_points, to_points, 100)
    
    def calcHomographyByAutoRANSACUsingIndex(self, idx_from, idx_to):
        from_img = self.images[idx_from]
        to_img = self.images[idx_to]
        
        return self.autoRansac(from_img.img_array, to_img.img_array)
        
    '''
    Use the ransac algorithm to choose the best set of points to use for the homography matrix
    '''
    def ransac(self, points1, points2, numiters):
        
        besthomography = np.zeros((3,3))
        besterror = -1
        
        #run numiter times and chose the best calculated homography
        for iters in range(numiters):
        
            #Randomly take a set of 4 points
            ranchoices1 = []
            ranchoices2 = []
            ranchoicepoints = []
            numpoints = 0
            print ("Size of points1: " + str(len(points1)))
            print ("Size of points2: " + str(len(points2)))            
            while numpoints < 4:
                choice = np.random.randint(0,len(points1)-1)
                print ("Choice: " + str(choice))
                pt1 = points1[choice]
                pt2 = points2[choice]
                if not (choice in ranchoicepoints):
                    print ("Size of ranpoints: " + str(len(ranchoicepoints)))
                    ranchoicepoints.append(choice)
                    ranchoices1.append(pt1)
                    ranchoices2.append(pt2)
                    numpoints += 1
                    
            #Calculate homography of said points
            matrix = self.calcHomograph(ranchoices1, ranchoices2)
            
            totalsum = 0
            
            #use homography to calculate mapping position of other points
            #and sum up error of calculated positions
            for i in range(len(points1)):
                a = np.ones((3,1))
                a[0,0] = points1[i][0]
                a[1,0] = points1[i][1]
                b = np.dot(matrix, a)
                totalsum += self.distance(b[0,0], points2[i][0], b[1,0], points2[i][1])
                
            #save the best homography
            if totalsum < besterror or besterror < 0:
                besterror = totalsum
                besthomography = matrix
                                
        return besthomography

    def ransac2(self, img1,  points1, img2, points2, numiters, threshold = 20):
        
        besthomography = np.zeros((3,3))
        max_inlinear_num = -1
        max_inlinear_set = []
        
        np.savetxt('p1.txt',points1, fmt="%d")
        np.savetxt('p2.txt',points2, fmt="%d")
        
        #run numiter times and chose the best calculated homography
        for iters in range(numiters):
            print "RANSAC @ " + str(iters)
        
        
            #Randomly take a set of 4 points
            ranchoices1 = []
            ranchoices2 = []
            ranchoicepoints = []
            numpoints = 0
            #print ("Size of points1: " + str(len(points1)))
            #print ("Size of points2: " + str(len(points2)))            
            while numpoints < 4:
                choice1 = np.random.randint(0,len(points1)-1)
                choice2 = np.random.randint(0, len(points2)-1)
                #print ("Choice: " + str(choice))
                pt1 = points1[choice1]
                pt2 = points2[choice2]
                Found = False
                for p1 in ranchoices1:
                    if pt1[0]==p1[0] and pt1[1]==p1[1]:
                        Found = True
                if Found == False:
                    for p2 in ranchoices2:
                        if pt2[0]==p2[0] and pt2[1]==p2[1]:
                            Found = True
                    
                if Found == False:
                    #print ("Size of ranpoints: " + str(len(ranchoicepoints)))
                    #ranchoicepoints.append(choice)
                    ranchoices1.append([pt1[0],pt1[1]])
                    ranchoices2.append([pt2[0],pt2[1]])
                    numpoints += 1
                    
            #Calculate homography of said points
            H = self.calcHomograph(ranchoices1, ranchoices2)
            
            inliner_num = 0
            inlinear_set = []
            #use homography to calculate mapping position of other points
            #and sum up error of calculated positions
            
            
            for i in range(len(points1)):
                '''
                a = np.ones((3,1))
                a[0,0] = points1[i][0]
                a[1,0] = points1[i][1]
                b = np.dot(matrix, a)
                dist = self.distance(b[0,0]/b[2,0], points2[i][0], b[1,0]/b[2,0], points2[i][1])
                '''
                scale = float(H[2,0]*points1[i][0]+H[2,1]*points1[i][1]+1.0)
                val_x = (H[0,0]*points1[i][0]+H[0,1]*points1[i][1]+H[0,2]) / scale
                val_y = (H[1,0]*points1[i][0]+H[1,1]*points1[i][1]+H[1,2]) / scale
                dist = self.distance(val_x, points2[i][0], val_y, points2[i][1])
                #dist = getNCC(img2, [val_x, val_y], img2, points2[i], 10)
                if dist < threshold:
                #if dist > 0.7:
                    inliner_num += 1
                    #print "Appending " + str(points1[i]) + " " + str(points2[i])
                    inlinear_set.append([[points1[i][0], points1[i][1]], [points2[i][0], points2[i][1]]])
            
                #print "inliner num " + str(inliner_num)
                    
            import copy
            #save the best homography
            if inliner_num > max_inlinear_num:
                max_inlinear_num = inliner_num
                best_inlinear_set = inlinear_set #copy.deepcopy(inlinear_set)
                
        #print best_inlinear_set 
        
        print "max_inlinear_num is " + str(max_inlinear_num)
        
        best_from_points = []
        best_to_points = []
        for p in best_inlinear_set:
            best_from_points.append(p[0])
            best_to_points.append(p[1])
        
        besthomography = self.calcHomographyByRegression(best_from_points, best_to_points)
        #besthomography = self.calcHomograph(best_from_points[0:4], best_to_points[0:4])
                                
        return besthomography, best_inlinear_set

    def autoRansac(self, img1, img2):
        hcd = HarrisCornerDetector(3 ,0.2, 100)
        points1 = hcd.getCornerPoints(img1)
        points2 = hcd.getCornerPoints(img2)
        window_size = 10
        numiters = 2000
        #points1 = []
        #points2 = []
        
        matchedpoints1, matchedpoints2 = prematch(img1, points1, img2, points2, 10)
        
        points_num = len(matchedpoints1)
        rndColors = []
        for i in range(points_num):
            rndVals = np.random.randint(0, 255,3)
            rndColors.append( (rndVals[0], rndVals[1], rndVals[2]) )
            
        point_radius = 20
        cimg1 = cv2.cvtColor(img1,cv2.COLOR_GRAY2BGR)
        cimg2 = cv2.cvtColor(img2,cv2.COLOR_GRAY2BGR)
        for i in range(points_num):
            p1 = matchedpoints1[i]
            p2 = matchedpoints2[i]
            print str(p1) + " -- " + str(p2)
            cv2.circle(cimg1,(p1[0]-point_radius,p1[1]-point_radius),point_radius,rndColors[i],point_radius)
            cv2.circle(cimg2,(p2[0]-point_radius,p2[1]-point_radius),point_radius,rndColors[i],point_radius)
        #cv2.imshow(filename1, cimg1)
        cv2.imwrite("img1_l.jpg", cimg1)
        #cv2.imshow(filename2, cimg2)
        cv2.imwrite("img2_l.jpg", cimg2)
        
        print "prematched size " + str(len(matchedpoints1))
        
        H, best_set =  self.ransac2(img1, matchedpoints1, img2, matchedpoints2, numiters)
        #H = self.ransac(matchedpoints1, matchedpoints2, numiters)
        
        points_num = len(best_set)
        rndColors = []
        for i in range(points_num):
            rndVals = np.random.randint(0, 255,3)
            rndColors.append( (rndVals[0], rndVals[1], rndVals[2]) )
            
        point_radius = 20
        cimg1 = cv2.cvtColor(img1,cv2.COLOR_GRAY2BGR)
        cimg2 = cv2.cvtColor(img2,cv2.COLOR_GRAY2BGR)
        for i in range(points_num):
            p1 = best_set[i][0]
            p2 = best_set[i][1]
            print str(p1) + " -- " + str(p2)
            cv2.circle(cimg1,(p1[0]-point_radius,p1[1]-point_radius),point_radius,rndColors[i],point_radius)
            cv2.circle(cimg2,(p2[0]-point_radius,p2[1]-point_radius),point_radius,rndColors[i],point_radius)
        #cv2.imshow(filename1, cimg1)
        cv2.imwrite("img1_l_best.jpg", cimg1)
        #cv2.imshow(filename2, cimg2)
        cv2.imwrite("img2_l_best.jpg", cimg2)
        
        return H
    
       
    def distance(self, x1, x2, y1, y2):
        return np.sqrt((x2-x1)**2+(y2-y1)**2)
    