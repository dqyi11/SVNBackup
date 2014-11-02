'''
Created on 2014-11-2

@author: Walter
'''

from PixelGraph import *
from ShapeClassifier import *

if __name__ == '__main__':
    
    img1_file = "shapes.pgm"
    img1 = cv2.imread(img1_file, 0)
    print img1.shape

    img1_bi = 1*(img1 >= 100)
    
    pxg1 = PixelGraph(img1_bi)
    
    cluster_num = 5
    feature_num = 2
    shpXfier = ShapeClassifier(feature_num, cluster_num)
    
    for s in pxg1.shapeDescriptors:
        shpXfier.addShape(s)
        
    shpXfier.classify()
    
    colors = [(255,0,0), (0,255,0), (0,0,255),(255,122,0),(0,122,122)]
    pxg1.visualizeByLabel("pxg1", colors)
    
    