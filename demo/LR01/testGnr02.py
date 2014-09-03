#https://grey.colorado.edu/emergent/index.php/Regression_with_MLP

from RegressionDataGenerator import *

if __name__ == '__main__':
    
    def func(x):
        return 1.2 * x[0] -3.4 * x[1] + 2.3 * x[2] + 5.7 * x[3]
    
    gnr = RegressionDataGenerator([4,1], func)
    gnr.geneData(100, 0.2)
    gnr.dump("data2")