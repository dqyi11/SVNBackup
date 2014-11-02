'''
Created on 2014-11-2

@author: Walter
'''

from PixelGraph import *

if __name__ == '__main__':
    
    img1_file = "shapes.pgm"
    img1 = cv2.imread(img1_file, 0)
    print img1.shape

    img1_bi = 1*(img1 >= 100)
    
    pxg1 = PixelGraph(img1_bi)
    