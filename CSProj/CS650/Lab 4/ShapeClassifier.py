'''
Created on 2014-11-2

@author: Walter
'''
import numpy as np
from kmean import *

class ShapeClassifier(object):

    def __init__(self, feature_dim, cluster_num):
        self.shapes = []
        self.feature_dim = feature_dim
        self.cluster_num = cluster_num

    def addShape(self, shape):
        self.shapes.append(shape)
        
    def classify(self):
        
        self.feature_vec = []
        self.shape_label = np.zeros(len(self.shapes), np.int)
        for s in self.shapes:
            self.feature_vec.append(s.getFeatureVector())
            
        km = KMeanCluster(self.feature_dim, self.cluster_num)
        km.cluster(self.feature_vec)
        
        for i in range(len(self.shapes)):
            self.shape_label[i] = km.getLabel(self.feature_vec[i])
            self.
            
    
        
            