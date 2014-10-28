'''
Created on Oct 27, 2014

@author: daqing_yi
'''

from PixelGraph import *
import cv2

if __name__ == '__main__':
    
    img_file = "shapes.pgm"
    img = cv2.imread(img_file, 0)
 
    graph = PixelGraph(img)
    
    '''
    remap = {v: i for i ,v in enumerate(unique(graph.labelData))}
    assert remap [0]==0
    for i in range (1, graph.labelData.shape[0]):
        for j in range (1 , graph.labelData.shape[1]):
            graph.labelData[i,j] = remap[graph.labelData[i,j]]
       
    cv2.imshow( recolor(graph.labelData), cmap = cm.spectral )
    '''       