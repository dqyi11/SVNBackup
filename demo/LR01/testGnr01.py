#https://grey.colorado.edu/emergent/index.php/Regression_with_MLP

from RegressionDataGenerator import *

if __name__ == '__main__':
    
    def func(x):
        return np.sin(2*np.pi*x) + np.cos(4*np.pi*x)
    
    gnr = RegressionDataGenerator([1,1], func)
    gnr.geneData(100, 0.04)
    gnr.dump("mlp_reg")