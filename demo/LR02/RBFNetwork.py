import numpy as np

class RBFNetwork(object):


    def __init__(self, dim, node_num, kernel_func):
        
        self.dim = dim
        self.node_num = node_num
        self.kernel_func = kernel_func
        
        self.weight_num = node_num
        self.center_num = node_num * dim
        
        self.beta_num = self.weight_num + self.center_num
        
    def calcFunc(self, betas, input):
        
        output = 0.0
        for i in range(self.node_num):
            center = np.zeros(self.dim)
            for d in range(self.dim):
                center[d] = betas[self.weight_num + i * self.dim + d]
            output += betas[i] * self.kernel_func(input, center)        
            
        return output