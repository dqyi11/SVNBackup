'''
Created on Nov 12, 2014

@author: daqing_yi
'''

from PyQt4 import QtCore, QtGui
from ImageManager import *
from ImageViewerConfigForm import *
from HarrisCornerDetector import *
from NCCMatching import *
import json
import cv2
import copy

class ImageView(QtGui.QMainWindow):

    def __init__(self, name, pixmap, label_points, label_colors, parent):
        super(ImageView, self).__init__(parent)
        self.name = name
        self.points = label_points
        self.pixmap = QtGui.QPixmap.fromImage(pixmap.toImage())

        qp = QtGui.QPainter()
        qp.begin(self.pixmap)
        
        inner_size = [10, 10]
        outer_size = [16, 16]
        
        white_pen = QtGui.QPen(QtGui.QColor(255,255,255))
        white_brush = QtGui.QBrush(QtGui.QColor(255,255,255))
        for p in self.points:
            pen = QtGui.QPen(label_colors[p["id"]-1])
            brush = QtGui.QBrush(label_colors[p["id"]-1])
            #pen.setWidth(10)        
            qp.setPen(white_pen)
            qp.fillRect(p["x"]-outer_size[0]/2, p["y"]-outer_size[1]/2, outer_size[0], outer_size[1], QtGui.QColor(255,255,255))
            
            qp.setPen(pen)
            qp.fillRect(p["x"]-inner_size[0]/2, p["y"]-inner_size[1]/2, inner_size[0], inner_size[1], label_colors[p["id"]-1])
        qp.end()
        
        
        self.label = QtGui.QLabel(self)
        self.scrollArea = QtGui.QScrollArea()
        self.scrollArea.setWidget(self.label)
        self.label.setPixmap(self.pixmap)
        self.label.resize(self.pixmap.size())
        self.setCentralWidget(self.scrollArea)
        self.setWindowTitle(self.name)
        
        
class ImageViewerWindow(QtGui.QMainWindow):

    def __init__(self):
        super(ImageViewerWindow, self).__init__()
        
        self.mdiArea = QtGui.QMdiArea()
        self.mdiArea.show()
        self.setCentralWidget(self.mdiArea)
        
        self.createActions()
        self.createMenus()
        
        self.fromIdx = 1
        self.toIdx = 0
        self.configWindow = ImageViewerConfigForm(self)
        self.imageDataList = []

        self.setWindowTitle("Image Viewer")
        self.show()
        
            
    def open(self):
        fileName = QtGui.QFileDialog.getOpenFileName(self)
        if fileName:
            with open(str(fileName)) as fp:
                jsondict = json.load(fp)
                
                self.im = ImageManager(jsondict)
                for i in range(self.im.image_num):
                    self.createSubImageViewer(self.im.images[i].id, self.im.images[i].pixmap, self.im.images[i].labeled_pixels, self.im.label_colors)                    
                    
                    
    def save(self):
        print "Save clicked"
        active_child = self.mdiArea.activeSubWindow()
        if active_child != None:
            active_child.widget().pixmap.save(active_child.widget().name+".png")
        
        
    def saveAll(self):
        print "Save all clicked"
        for child in self.mdiArea.subWindowList(order=QtGui.QMdiArea_CreationOrder):
            child.widget().pixmap.save(child.widget().name+".png")
            
    def calcHomograph(self):
        print "Calc Homography"

        self.configWindow.exec_()
        print "From " + str(self.fromIdx) + " to " + str(self.toIdx)
        H = self.im.calcHomographyUsingIndex(self.fromIdx, self.toIdx)
        H_r = self.im.calcHomographyByRegressionUsingIndex(self.fromIdx, self.toIdx)
        
        print H
        print H_r
        
        print "Error "
        print str(H-H_r)
        
        '''
        print "Homography"
        for i in range(3):
            for j in range(3):
                print 'H' + str(j) + str(j)
                print '%0.2f' % H[i,j]
                print '%0.2f' % H_r[i,j]
        '''

        
    def warpImage(self):
        print "Warp image"
        
        self.configWindow.exec_()
        print "From " + str(self.fromIdx) + " to " + str(self.toIdx)
        self.mapImage(self.fromIdx, self.toIdx)
        
    def stitch(self, stitch_all=False):
        print "Stitch images"
        Hs = []
        mapped_imgs = []
        transs = []
        self.imageDataList = []
        
        if stitch_all==True:
        
            toImg = self.im.images[0].pixmap.toImage()
            self.imageDataList.append((toImg, (0,0)))
            for i in range(1,self.im.image_num):
                
                H = self.im.calcHomographyByAutoRANSACUsingIndex(i, 0)
                #H = self.im.calcHomographyUsingIndex(i, 0)
                fromImg = self.im.images[i].pixmap.toImage()
                
                mapped_from_img, from_trans = self.im.bilinearInterpolation(fromImg, H)
                qpix = QtGui.QPixmap.fromImage(mapped_from_img)
                self.createSubImageViewer(str(i)+" and "+str(0), qpix, [], [])
                Hs.append(H)
                mapped_imgs.append(mapped_from_img)
                transs.append(from_trans)
                self.imageDataList.append((mapped_from_img, from_trans))
                
                blended_img = self.blendImages(self.imageDataList, False)
                
                # createSubImageViewer
                qpix = QtGui.QPixmap.fromImage(blended_img)
                windName = str(self.fromIdx) + " and " + str(self.toIdx)
                self.createSubImageViewer(windName, qpix, [], [])
                
        else:
            
            if len(self.imageDataList) <= 2:
                self.configWindow.exec_()
                print "From " + str(self.fromIdx) + " to " + str(self.toIdx)

                H = self.im.calcHomographyByAutoRANSACUsingIndex(self.fromIdx, self.toIdx)
                #H = self.im.calcHomographyUsingIndex(self.fromIdx, self.toIdx)
                #H = self.im.calcHomographyUsingRegression(fromIdx, toIdx)
                # call bilinearInterpolation
                
            fromImg = self.im.images[self.fromIdx].pixmap.toImage()
            toImg = self.im.images[self.toIdx].pixmap.toImage()
            mapped_from_img, from_trans = self.im.bilinearInterpolation(fromImg, H)
            
            self.imageDataList.append((toImg, (0,0)))
            self.imageDataList.append((mapped_from_img, from_trans))
    
            blended_img = self.blendImages(self.imageDataList, False)
            
            # createSubImageViewer
            qpix = QtGui.QPixmap.fromImage(blended_img)
            windName = str(self.fromIdx) + " and "
            windName += str(self.toIdx)
            self.createSubImageViewer(windName, qpix, [], [])

    def auto_ransac(self):
        print "Auto Ransac"
        
        self.configWindow.exec_()
        print "From " + str(self.fromIdx) + " to " + str(self.toIdx)
        H = self.im.calcHomographyByAutoRANSACUsingIndex(self.fromIdx, self.toIdx)
        
        H_r = self.im.calcHomographyUsingIndex(self.fromIdx, self.toIdx)
        print "H"
        print H
        print "H_r"
        print H_r
        
        print "Erorr"
        print str(H-H_r)
        
    def ransac(self):
        print "Ransac"
        
        self.configWindow.exec_()
        print "From " + str(self.fromIdx) + " to " + str(self.toIdx)
        H = self.im.calcHomographyByRANSACUsingIndex(self.fromIdx, self.toIdx)
        
        H_r = self.im.calcHomographyUsingIndex(self.fromIdx, self.toIdx)
        print "H"
        print H
        print "H_r"
        print H_r
        
    def mapImage(self, fromIdx, toIdx):
        
        # calculate homograph
        H = self.im.calcHomographyUsingIndex(fromIdx, toIdx)
        #H = self.im.calcHomographyUsingRegression(fromIdx, toIdx)
        # call bilinearInterpolation
        
        fromImg = self.im.images[fromIdx].pixmap.toImage()
        qimg, trans = self.im.bilinearInterpolation(fromImg, H)
        
        toImg = self.im.images[self.toIdx].pixmap.toImage()
        self.imageDataList.append((toImg, (0,0)))
        self.imageDataList.append((qimg, trans))
        
        # createSubImageViewer
        qpix = QtGui.QPixmap.fromImage(qimg)
        self.createSubImageViewer(str(fromIdx)+" to "+str(toIdx), qpix, [], [])
        

        
    def blendImages(self, imageDataList, enableWeight=False):
        # image ( img, trans )
        
        xlist = []
        ylist = []
        for imageData in imageDataList:
            img = imageData[0]
            trans = imageData[1]
            xlist.append(trans[0])
            xlist.append(trans[0]+img.width())
            ylist.append(trans[1])
            ylist.append(trans[1]+img.height())
            
        minX = min(xlist)
        maxX = max(xlist)
        minY = min(ylist)
        maxY = max(ylist)
        
        blendedImage = QtGui.QImage(maxX-minX, maxY-minY, QtGui.QImage.Format_ARGB32)
        
        for x in range(minX, maxX+1):
            if x%50==0:
                print "processing " + str(x) + "/" + str(maxX)
            for y in range(minY, maxY+1):
                r_vals, g_vals, b_vals = [], [], []
                dist_vals = []
                dist_sum = 0.0
                for imageData in imageDataList:
                    img = imageData[0]
                    trans = imageData[1]
                    if x >= trans[0] and y >= trans[1] and x < trans[0]+img.width() and y < trans[1]+img.height():
                        val = QtGui.QColor(img.pixel(x-trans[0], y-trans[1]))
                        if val.alpha() > 0:
                            if enableWeight == True:
                                dist = np.sqrt(float((x-(trans[0]+img.width())/2)**2)+float((y-(trans[1]+img.height())/2)**2))
                                dist_vals.append((dist))
                                dist_sum += dist
                            r_vals.append(val.red())
                            g_vals.append(val.green())
                            b_vals.append(val.blue())
                
                if len(r_vals) > 0:
                    if enableWeight == True:
                        dist_vals /= dist_sum
                        avg_r = int(np.mean(np.array(dist_vals)*np.array(r_vals)))
                        avg_g = int(np.mean(np.array(dist_vals)*np.array(g_vals)))
                        avg_b = int(np.mean(np.array(dist_vals)*np.array(b_vals)))
                    else:
                        avg_r = int(np.mean(r_vals))
                        avg_g = int(np.mean(g_vals))
                        avg_b = int(np.mean(b_vals))
                    avg_val = QtGui.QColor(avg_r, avg_g, avg_b)
                    blendedImage.setPixel(QtCore.QPoint(x-minX,y-minY), avg_val.rgba())
        
        return blendedImage
                
    def createActions(self):
        self.openAction = QtGui.QAction("Open", self)
        self.openAction.triggered.connect(self.open)
        
        self.saveAction = QtGui.QAction("Save", self)
        self.saveAction.triggered.connect(self.save)
        
        self.homographyAction = QtGui.QAction("Calc Homograph", self)
        self.homographyAction.triggered.connect(self.calcHomograph)
        
        self.warpImgAction = QtGui.QAction("Warp Image", self)
        self.warpImgAction.triggered.connect(self.warpImage)
        
        self.stitchAction = QtGui.QAction("Stitch Images", self)
        self.stitchAction.triggered.connect(self.stitch)
        
        self.ransacAction = QtGui.QAction("Ransac", self)
        self.ransacAction.triggered.connect(self.ransac)
        
        self.ransacAutoAction = QtGui.QAction("Auto Ransac", self)
        self.ransacAutoAction.triggered.connect(self.auto_ransac)
    
    def createMenus(self):
        self.fileMenu = self.menuBar().addMenu("&File")
        self.fileMenu.addAction(self.openAction)
        self.fileMenu.addAction(self.saveAction)
        
        self.fileMenu = self.menuBar().addMenu("&Edit")
        self.fileMenu.addAction(self.homographyAction)
        self.fileMenu.addAction(self.warpImgAction)
        self.fileMenu.addAction(self.stitchAction)
        self.fileMenu.addAction(self.ransacAction)
        self.fileMenu.addAction(self.ransacAutoAction)
         
    def createSubImageViewer(self, name, pixmap, label_points, label_colors):
        child = ImageView(name, pixmap, label_points, label_colors, self.mdiArea)
        self.mdiArea.addSubWindow(child)
        child.show()
        return child       
        

        
        
        