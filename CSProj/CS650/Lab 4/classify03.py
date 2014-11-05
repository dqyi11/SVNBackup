'''
Created on Nov 2, 2014

@author: daqing_yi
'''

from PixelGraph import *
from ShapeClassifier import *

if __name__ == '__main__':
    
    img1_file = "train1.pgm"
    img2_file = "train2.pgm"
    img1 = cv2.imread(img1_file, 0)
    img2 = cv2.imread(img2_file, 0)
    print img1.shape
    print img2.shape

    img1_bi = 1*(img1 <= 100)
    img2_bi = 1*(img2 <= 100)
    
    pxg1 = PixelGraph(img1_bi)
    pxg2 = PixelGraph(img2_bi)
    
    cluster_num = 5
    feature_num = 15
    feature_weights = np.ones(feature_num)
    feature_weights[0] = 3.0
    feature_weights[1] = 2.0
    feature_weights[5] = 3.0
    feature_weights[7] = 2.0
    shpXfier = ShapeClassifier(feature_num, cluster_num, feature_weights)
    
    for s in pxg1.shapeDescriptors:
        shpXfier.addShape(s)
    for s in pxg2.shapeDescriptors:
        shpXfier.addShape(s)
        
    shpXfier.classify()
    
    colors = [(255,0,0), (0,255,0), (0,0,255),(255,122,0),(0,122,122)]
    pxg1.visualizeByLabel(img1_file, colors)
    pxg2.visualizeByLabel(img2_file, colors)