'''
Created on Oct 27, 2014

@author: daqing_yi
'''

from PixelGraph import *
import cv2

if __name__ == '__main__':
    
    #img_file = "simplecircles.ppm"
    img_file = "shapes.pgm"
    img = cv2.imread(img_file, 0)
    print img.shape

    img_bi = 1*(img >= 100)
    
    graph = PixelGraph(img_bi)
    
    print graph.componentMgr.getComponentNum()
    
    #graph.dump(img_file+'.csv')
    #graph.visualize(img_file)
    
    '''
    print "perimeter " + str(graph.shapeDescriptors[2].getPerimeter())
    graph.visualizeComponentBoundary(img_file, 2)
    graph.visualizeComponentChainCode(img_file, 2)
    '''
    #print str(graph.shapeDescriptors[2].getConvexHull())
    #graph.visualizeConvexHull(img_file, 6)
    
    #print graph.shapeDescriptors[3].getMinimumBoundingRectangle()
    graph.visualizeMinimumBoundingRect(img_file, 3)

    
    