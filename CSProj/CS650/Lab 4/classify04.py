'''
Created on Nov 2, 2014

@author: daqing_yi
'''

from PixelGraph import *
from ShapeClassifier import *

if __name__ == '__main__':
    
    img1_file = "train1.pgm"
    img2_file = "train2.pgm"
    img3_file = "match1.pgm"
    img4_file = "match2.pgm"
    img1 = cv2.imread(img1_file, 0)
    img2 = cv2.imread(img2_file, 0)
    img3 = cv2.imread(img3_file, 0)
    img4 = cv2.imread(img4_file, 0)
    print img1.shape
    print img2.shape
    print img3.shape
    print img4.shape

    img1_bi = 1*(img1 <= 100)
    img2_bi = 1*(img2 <= 100)
    img3_bi = 1*(img3 <= 100)
    img4_bi = 1*(img4 <= 100)
    
    pxg1 = PixelGraph(img1_bi)
    pxg2 = PixelGraph(img2_bi)
    pxg3 = PixelGraph(img3_bi)
    pxg4 = PixelGraph(img4_bi)
    
    cluster_num = 5
    feature_num = 2
    shpXfier = ShapeClassifier(feature_num, cluster_num)
    
    for s in pxg1.shapeDescriptors:
        shpXfier.addShape(s)
    for s in pxg2.shapeDescriptors:
        shpXfier.addShape(s)
        
    shpXfier.classify()
    
    colors = [(255,0,0), (0,255,0), (0,0,255),(255,122,0),(0,122,122)]
    pxg1.visualizeByLabel(img1_file, colors)
    pxg2.visualizeByLabel(img2_file, colors)
    
    for s in pxg3.shapeDescriptors:
        s.label = shpXfier.getLabel(s)
    for s in pxg4.shapeDescriptors:
        s.label = shpXfier.getLabel(s)
    
    
    pxg3.visualizeByLabel(img3_file, colors)
    pxg4.visualizeByLabel(img4_file, colors)