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
            
        self.km = KMeanCluster(self.feature_dim, self.cluster_num)
        self.km.cluster(self.feature_vec)
        
        for i in range(len(self.shapes)):
            dist, self.shape_label[i] = self.km.getLabel(self.feature_vec[i])
            self.shapes[i].label = self.shape_label[i]
            
    def getLabel(self, shape):
        dist, label = self.km.getLabel(shape.getFeatureVector())
        return label
            
    
        
            